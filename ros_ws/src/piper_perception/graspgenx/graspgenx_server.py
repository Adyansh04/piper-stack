"""Serves GraspGenX parallel-gripper grasps for one object's points over ZMQ.

Run it in its venv with its parameter file, as ``graspgenx_server.launch.py`` does::

    /opt/graspgenx-venv/bin/python graspgenx_server.py --params config/graspgenx_server.yaml

Python because GraspGenX's point-cloud backbones (PTv3, PointNet++) have no TensorRT or TorchScript
path, so the model runs in PyTorch; a ZMQ server and not a ROS node because GraspGenX needs Python
3.12 and Lyrical's rclpy is 3.14. The ``grasp_generation`` node serves ``compute_grasps`` on the ROS
graph and forwards each request here. The gripper is described to GraspGenX by its open and
half-open finger sweep boxes alone, from the parameter file.

Wire protocol, ZMQ REQ/REP, little-endian, no pickle:

- request: ``<II`` (protocol version, grasps wanted; 0 asks for the configured number), then the
  points as float32 x, y, z, metres, in a frame whose z points up;
- reply: ``<II`` (protocol version, success 0 or 1), the message (UTF-8), then the grasps, best
  first, as float64 rows ``x y z qx qy qz qw score width_m object_width_m``: the TCP pose in the
  points' frame, the discriminator's score, the opening the grasp needs and the object's width
  between the fingers. The rows are empty when success is 0.
"""

from __future__ import annotations

import argparse
import logging
import os
import struct
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import numpy as np
import yaml
import zmq
from frames import measure_finger_extents, to_tcp_poses
from scipy.spatial.transform import Rotation

PROTOCOL_VERSION = 1
#: Version and grasps wanted (request) or version and success (reply).
HEADER = struct.Struct('<II')
#: Columns of a reply row: position, quaternion x y z w, score, width_m, object_width_m.
REPLY_COLUMNS = 10

_log = logging.getLogger('graspgenx_server')

Box = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class ServerConfig:
    """The server's parameters, from the ``graspgenx_server`` section of its YAML.

    The YAML documents each one. Boxes and offsets are in GraspGenX's gripper frame (+Z the
    approach from the gripper base, +X the closing direction), metres.
    """

    endpoint: str
    checkpoint_dir: Path
    extents_open_m: Box
    offset_open_m: Box
    extents_mid_m: Box
    offset_mid_m: Box
    finger_thickness_m: float
    fingertip_depth_m: float
    tcp_depth_m: float
    gripper_type: int
    planner: str
    moe_z_offsets_cm: tuple[float, ...]
    num_grasps: int
    min_confidence: float
    max_grasps: int
    min_points: int
    max_points: int
    idle_unload_s: float
    idle_check_s: float
    preload: bool
    load_retry_s: float

    @classmethod
    def from_yaml(cls, path: Path) -> ServerConfig:
        """Read and check the parameters.

        Args:
            path: The parameter YAML, with every parameter under ``graspgenx_server``.

        Returns:
            The checked parameters.

        Raises:
            ValueError: If a parameter is missing, unknown or out of range, ``checkpoint_dir`` lacks
                ``gen/`` or ``dis/``, a box is off the closing axis's zero or has not three
                numbers, or ``planner`` is unknown (GraspGenX takes any other name for its
                diffusion baseline).
        """
        values = (yaml.safe_load(path.read_text()) or {}).get('graspgenx_server')
        if not isinstance(values, dict):
            raise ValueError(f'{path} has no graspgenx_server section')
        names = {field.name for field in fields(cls)}
        if unknown := sorted(set(values) - names):
            raise ValueError(f'{path}: unknown parameters {unknown}')
        if missing := sorted(names - set(values)):
            raise ValueError(f'{path}: missing parameters {missing}')
        boxes = {}
        for name in ('extents_open_m', 'offset_open_m', 'extents_mid_m', 'offset_mid_m'):
            box = tuple(float(v) for v in values[name])
            if len(box) != 3:
                raise ValueError(f'{name} is {values[name]}: three numbers, x y z')
            boxes[name] = box
        for name in ('offset_open_m', 'offset_mid_m'):
            if boxes[name][0] != 0.0:
                raise ValueError(
                    f"{name} is {list(boxes[name])}: the fingers close about the gripper base's z "
                    'axis, so the box is centred on it (x 0)'
                )
        config = cls(
            endpoint=str(values['endpoint']),
            checkpoint_dir=Path(values['checkpoint_dir']),
            **boxes,
            finger_thickness_m=float(values['finger_thickness_m']),
            fingertip_depth_m=float(values['fingertip_depth_m']),
            tcp_depth_m=float(values['tcp_depth_m']),
            gripper_type=int(values['gripper_type']),
            planner=str(values['planner']),
            moe_z_offsets_cm=tuple(float(v) for v in values['moe_z_offsets_cm']),
            num_grasps=int(values['num_grasps']),
            min_confidence=float(values['min_confidence']),
            max_grasps=int(values['max_grasps']),
            min_points=int(values['min_points']),
            max_points=int(values['max_points']),
            idle_unload_s=float(values['idle_unload_s']),
            idle_check_s=float(values['idle_check_s']),
            preload=bool(values['preload']),
            load_retry_s=float(values['load_retry_s']),
        )
        for part in ('gen', 'dis'):
            if not (config.checkpoint_dir / part).is_dir():
                raise ValueError(
                    f"checkpoint_dir {config.checkpoint_dir} has no {part}/: mount GraspGenX's "
                    'release checkpoints there'
                )
        if config.planner not in ('graspmoe', 'diffusion'):
            raise ValueError(f"planner '{config.planner}' is not graspmoe or diffusion")
        if config.num_grasps < 1:
            raise ValueError(f'num_grasps {config.num_grasps} is below 1')
        if config.max_points < config.min_points:
            raise ValueError(
                f'max_points {config.max_points} is below min_points {config.min_points}'
            )
        if config.idle_check_s <= 0.0:
            raise ValueError(f'idle_check_s {config.idle_check_s} is not positive')
        return config


class GraspModel:
    """GraspGenX's sampler, loaded on demand and freed after ``idle_unload_s`` without a request.

    One thread uses it.

    Args:
        config: The server's parameters.
    """

    def __init__(self, config: ServerConfig) -> None:
        self.config = config
        #: The loaded sampler; None until the first load and after an idle unload.
        self.sampler: Any = None
        #: GraspGenX's planner and torch, imported by the first load.
        self.run_planner: Callable[..., Any] | None = None
        self.torch: Any = None
        #: Why the last load failed and when, time.monotonic() seconds; None after a good one.
        self.load_error: str | None = None
        self.load_failed_s = 0.0
        #: When a request last used the model, time.monotonic() seconds.
        self.last_used_s = time.monotonic()

    def ensure_loaded(self) -> None:
        """Load the model if it is not; for ``load_retry_s`` after a failure, raise that again.

        A load takes seconds of disk and GPU memory: requests must not each pay for one that fails.

        Raises:
            RuntimeError: If the last load failed less than ``load_retry_s`` ago.
        """
        if self.sampler is not None:
            return
        if (
            self.load_error is not None
            and time.monotonic() - self.load_failed_s < self.config.load_retry_s
        ):
            raise RuntimeError(f'the last load failed: {self.load_error}')
        try:
            self._load()
        except Exception as error:
            self.load_error = repr(error)
            self.load_failed_s = time.monotonic()
            raise
        self.load_error = None

    def _load(self) -> None:
        """Import GraspGenX and torch, and load the generator and discriminator."""
        # Both must name existing directories, or importing GraspGenX clones its checkpoints and
        # gripper descriptions from Hugging Face; the sweep volume below needs neither.
        os.environ.setdefault('GRASPGENX_CHECKPOINT_DIR', str(self.config.checkpoint_dir))
        os.environ.setdefault('GRASPGENX_GRIPPER_CFG_DIR', str(self.config.checkpoint_dir))
        import torch
        from graspgenx.grasp_server import GraspGenXSampler
        from graspgenx.samplers.planner import run_planner_on_object
        from graspgenx.utils.checkpoint_io import load_model_cfg
        from graspgenx.x_grippers import make_sweep_volume_gripper_info

        start = time.monotonic()
        config = self.config
        model_cfg = load_model_cfg(
            str(config.checkpoint_dir / 'gen'), str(config.checkpoint_dir / 'dis')
        )
        # The release checkpoints condition on the sweep volume alone ("sweep_volume_v2").
        gripper = make_sweep_volume_gripper_info(
            config.extents_open_m,
            config.offset_open_m,
            config.extents_mid_m,
            config.offset_mid_m,
            gripper_type=config.gripper_type,
            fingertip_depth=config.fingertip_depth_m,
            name='piper',
        )
        self.sampler = GraspGenXSampler(model_cfg, gripper_info=gripper)
        self.run_planner = run_planner_on_object
        self.torch = torch
        _log.info('GraspGenX loaded in %.1f s', time.monotonic() - start)

    def plan(self, points: np.ndarray, num_grasps: int) -> tuple[np.ndarray, np.ndarray]:
        """Run GraspGenX on an object's points.

        Args:
            points: Object points [N, 3], float32, metres, z up.
            num_grasps: Diffusion samples to draw.

        Returns:
            Gripper-base poses [K, 4, 4] and their scores [K], in no particular order.
        """
        self.ensure_loaded()
        config = self.config
        grasps, confidence = self.run_planner(
            points,
            self.sampler,
            planner=config.planner,
            grasp_threshold=config.min_confidence,
            num_grasps=num_grasps,
            moe_z_offsets_cm=list(config.moe_z_offsets_cm),
        )[:2]
        self.last_used_s = time.monotonic()
        return (
            np.asarray(grasps, dtype=np.float64).reshape(-1, 4, 4),
            np.asarray(confidence, dtype=np.float64).reshape(-1),
        )

    def warm_up(self) -> None:
        """Load the model and run it once on a box, so the first request pays for neither."""
        start = time.monotonic()
        try:
            box = np.random.default_rng(0).uniform(
                [-0.03, -0.03, 0.0], [0.03, 0.03, 0.05], (1000, 3)
            )
            self.plan(box.astype(np.float32), self.config.num_grasps)
        except Exception:  # noqa: BLE001 - a failed warm-up leaves loading to the first request
            _log.exception('GraspGenX warm-up failed')
            return
        _log.info('GraspGenX ready: loaded and run once in %.1f s', time.monotonic() - start)

    def unload_if_idle(self) -> None:
        """Free the model after ``idle_unload_s`` without a request (0: never)."""
        idle_s = self.config.idle_unload_s
        if self.sampler is None or idle_s <= 0.0 or time.monotonic() - self.last_used_s <= idle_s:
            return
        self.sampler = None
        self.torch.cuda.empty_cache()
        _log.info('GraspGenX idle for %.0f s: model freed', idle_s)

    def is_fatal(self, error: Exception) -> bool:
        """Whether an error leaves CUDA unusable for the process's life (out of memory does not)."""
        out_of_memory = self.torch is not None and isinstance(
            error, self.torch.cuda.OutOfMemoryError
        )
        return 'CUDA' in str(error) and self.sampler is not None and not out_of_memory


class RequestError(ValueError):
    """A request the server cannot read; answered with its message."""


def decode_request(frames: list[bytes]) -> tuple[int, np.ndarray]:
    """Read a request's frames.

    Args:
        frames: The header and the points.

    Returns:
        The grasps wanted (0: the configured number) and the points [N, 3], float32, read-only.

    Raises:
        RequestError: If the frames do not follow the protocol.
    """
    if len(frames) != 2:
        raise RequestError(f'{len(frames)} frames, 2 expected: the header and the points')
    if len(frames[0]) != HEADER.size:
        raise RequestError(f'a {len(frames[0])}-byte header, {HEADER.size} expected')
    version, num_grasps = HEADER.unpack(frames[0])
    if version != PROTOCOL_VERSION:
        raise RequestError(f'protocol version {version}, this server speaks {PROTOCOL_VERSION}')
    if len(frames[1]) % 12 != 0:
        raise RequestError(f'{len(frames[1])} bytes of points: not x, y, z float32 triples')
    return num_grasps, np.frombuffer(frames[1], dtype='<f4').reshape(-1, 3)


def encode_reply(success: bool, message: str, rows: np.ndarray | None = None) -> list[bytes]:
    """Return a reply's frames.

    Args:
        success: Whether the request succeeded.
        message: What happened, for the log and the operator.
        rows: The grasps [K, REPLY_COLUMNS], best first; None when success is false.
    """
    body = b'' if rows is None else np.ascontiguousarray(rows, dtype='<f8').tobytes()
    return [HEADER.pack(PROTOCOL_VERSION, int(success)), message.encode(), body]


class GraspServer:
    """Answers requests with GraspGenX's grasps, best first, in the TCP's frame convention.

    The points' frame must have z up, since GraspMoE's top-down grasps follow its -z. Of a cloud
    above ``max_points`` the model sees that many points, picked at random (GraspGenX's outlier
    removal builds an N x N distance matrix); the openings are measured on all of them.

    Args:
        config: The server's parameters.
        model: The model it runs.
    """

    def __init__(self, config: ServerConfig, model: GraspModel) -> None:
        self.config = config
        self.model = model
        #: Set when a request hit an error that leaves CUDA unusable: the server must restart.
        self.failed_fatally = False
        #: Picks the points the model sees of a large cloud; seeded, so runs repeat.
        self.rng = np.random.default_rng(0)

    def answer(self, frames: list[bytes]) -> list[bytes]:
        """Answer one request's frames; never raises, since a dropped reply strands the client."""
        try:
            return self._compute(*decode_request(frames))
        except RequestError as error:
            return encode_reply(False, f'Bad request: {error}')
        except Exception as error:  # noqa: BLE001 - any failure becomes the reply
            _log.exception('GraspGenX request failed')
            self.failed_fatally = self.model.is_fatal(error)
            return encode_reply(False, f'GraspGenX failed: {error!r}')

    def _compute(self, num_grasps: int, points: np.ndarray) -> list[bytes]:
        """Run the model on a decoded request and encode its reply."""
        config = self.config
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) < config.min_points:
            return encode_reply(
                False, f'{len(points)} object points, at least {config.min_points} needed'
            )
        if num_grasps > config.max_grasps:
            return encode_reply(
                False, f'num_grasps {num_grasps} is above max_grasps ({config.max_grasps})'
            )
        model_points = points
        if len(points) > config.max_points:
            model_points = points[self.rng.choice(len(points), config.max_points, replace=False)]
        start = time.monotonic()
        grasps, confidence = self.model.plan(model_points, num_grasps or config.num_grasps)
        if len(grasps) == 0:
            return encode_reply(
                False,
                f'no grasp above the confidence threshold {config.min_confidence}'
                if config.min_confidence > 0.0
                else f'GraspGenX returned no grasps for {len(model_points)} points: too sparse '
                'after its outlier removal?',
            )
        order = np.argsort(-confidence, kind='stable')[: config.max_grasps]
        grasps, confidence = grasps[order], confidence[order]
        openings, widths = measure_finger_extents(
            points,
            grasps,
            extents_open_m=np.asarray(config.extents_open_m),
            offset_open_m=np.asarray(config.offset_open_m),
            finger_thickness_m=config.finger_thickness_m,
        )
        tcp = to_tcp_poses(grasps, config.tcp_depth_m)
        rows = np.empty((len(tcp), REPLY_COLUMNS))
        rows[:, 0:3] = tcp[:, :3, 3]
        rows[:, 3:7] = Rotation.from_matrix(tcp[:, :3, :3]).as_quat()
        rows[:, 7] = confidence
        rows[:, 8] = openings
        rows[:, 9] = widths
        message = f'{len(rows)} grasps in {1000.0 * (time.monotonic() - start):.0f} ms'
        _log.info('Returned %s', message)
        return encode_reply(True, message, rows)


def serve(config: ServerConfig) -> int:
    """Answer requests until Ctrl-C, or until a CUDA error makes a restart necessary.

    Args:
        config: The server's parameters.

    Returns:
        The process's exit code: 0 after Ctrl-C, 1 after a CUDA error (launch respawns it).
    """
    model = GraspModel(config)
    server = GraspServer(config, model)
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    # The last reply gets a second to leave before the process ends.
    socket.setsockopt(zmq.LINGER, 1000)
    socket.bind(config.endpoint)
    _log.info('GraspGenX server on %s, model from %s', config.endpoint, config.checkpoint_dir)
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    try:
        # Requests that arrive meanwhile wait in the socket.
        if config.preload:
            model.warm_up()
        while True:
            if not poller.poll(int(config.idle_check_s * 1000.0)):
                model.unload_if_idle()
                continue
            socket.send_multipart(server.answer(socket.recv_multipart()))
            if server.failed_fatally:
                _log.critical('GraspGenX: a CUDA error, exiting for a restart')
                return 1
    except KeyboardInterrupt:
        return 0
    finally:
        socket.close()
        context.term()


def main(argv: list[str] | None = None) -> int:
    """Parse the arguments and serve; returns the exit code."""
    parser = argparse.ArgumentParser(description='GraspGenX grasps over ZMQ.')
    parser.add_argument(
        '--params', type=Path, required=True, help='Parameter YAML (graspgenx_server section).'
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='[graspgenx_server] %(levelname)s %(message)s')
    return serve(ServerConfig.from_yaml(args.params))


if __name__ == '__main__':
    sys.exit(main())

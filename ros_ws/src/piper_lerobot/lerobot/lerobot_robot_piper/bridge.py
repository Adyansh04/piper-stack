"""A client of piper_lerobot's bridge node: the arm's state and wrist image, and joint targets.

The protocol is the bridge's (``include/piper_lerobot/bridge_protocol.hpp``): ZMQ request-reply,
little-endian float64 positions, the image as rgb8 rows.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import zmq


@dataclass(frozen=True, slots=True)
class Observation:
    """One reply of the bridge to ``observe``."""

    #: Time of the joint readings, in s (the stack's clock).
    stamp_s: float
    #: Measured position of each joint: rad, then the gripper's finger travel in m.
    measured: np.ndarray
    #: Commanded position of each joint, in the same order and units.
    commanded: np.ndarray
    #: The wrist camera's colour image, height x width x 3, RGB, read-only.
    image: np.ndarray


class BridgeClient:
    """A request-reply client of the bridge.

    Args:
        endpoint: The bridge's ZMQ endpoint, such as ``tcp://127.0.0.1:5590``.
        timeout_s: How long a reply may take, in s.
    """

    def __init__(self, endpoint: str, timeout_s: float) -> None:
        self._endpoint = endpoint
        self._timeout_ms = int(timeout_s * 1000)
        self._context = zmq.Context.instance()
        self._socket: zmq.Socket | None = None
        self._open()

    def _open(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(self._endpoint)

    def _request(self, frames: list[bytes]) -> list[bytes]:
        self._socket.send_multipart(frames)
        if not self._socket.poll(self._timeout_ms):
            # A request socket without its reply cannot send again: start a new one.
            self._open()
            raise TimeoutError(
                f'No reply from the bridge at {self._endpoint} in {self._timeout_ms} ms'
            )
        reply = self._socket.recv_multipart()
        if reply[0] != b'ok':
            reason = reply[1].decode() if len(reply) > 1 else 'no reason given'
            raise RuntimeError(f'The bridge at {self._endpoint} refused: {reason}')
        return reply

    def observe(self) -> Observation:
        """Return the arm's latest state and wrist image.

        Raises:
            TimeoutError: If the bridge does not answer in time.
            RuntimeError: If the bridge has no joint state or image yet.
        """
        _, state_frame, size_frame, image_frame = self._request([b'observe'])
        state = np.frombuffer(state_frame, dtype='<f8')
        joints = (state.size - 1) // 2
        height, width = (int(v) for v in np.frombuffer(size_frame, dtype='<u4'))
        image = np.frombuffer(image_frame, dtype=np.uint8).reshape(height, width, 3)
        return Observation(
            stamp_s=float(state[0]),
            measured=state[1 : 1 + joints],
            commanded=state[1 + joints :],
            image=image,
        )

    def act(self, positions: list[float]) -> None:
        """Send joint targets: rad for the arm's joints, then m for the gripper.

        Raises:
            TimeoutError: If the bridge does not answer in time.
            RuntimeError: If the bridge refuses the targets (another count, a non-finite value).
        """
        self._request([b'act', np.asarray(positions, dtype='<f8').tobytes()])

    def release(self) -> None:
        """Stop the arm: the bridge sends no more joint jogs."""
        self._request([b'release'])

    def close(self) -> None:
        """Close the socket."""
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None

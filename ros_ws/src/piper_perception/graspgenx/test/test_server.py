"""The GraspGenX server's protocol and request handling, with GraspGenX's planner stubbed out."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from graspgenx_server import (
    HEADER,
    PROTOCOL_VERSION,
    REPLY_COLUMNS,
    GraspModel,
    GraspServer,
    ServerConfig,
)

PACKAGE = Path(__file__).resolve().parents[2]
BASE = {
    'endpoint': 'tcp://127.0.0.1:5561',
    'extents_open_m': [0.07, 0.024, 0.023],
    'offset_open_m': [0.0, 0.0, 0.1243],
    'extents_mid_m': [0.035, 0.024, 0.023],
    'offset_mid_m': [0.0, 0.0, 0.1243],
    'finger_thickness_m': 0.008,
    'fingertip_depth_m': 0.1358,
    'tcp_depth_m': 0.1295,
    'gripper_type': 0,
    'planner': 'graspmoe',
    'moe_z_offsets_cm': [-2.0, -1.0],
    'num_grasps': 200,
    'min_confidence': 0.0,
    'max_grasps': 2,
    'min_points': 50,
    'max_points': 100,
    'idle_unload_s': 0.0,
    'idle_check_s': 5.0,
    'preload': False,
    'load_retry_s': 30.0,
}


def write_config(tmp_path: Path, **overrides: Any) -> Path:
    (tmp_path / 'gen').mkdir(exist_ok=True)
    (tmp_path / 'dis').mkdir(exist_ok=True)
    path = tmp_path / 'params.yaml'
    values = {**BASE, 'checkpoint_dir': str(tmp_path), **overrides}
    path.write_text(yaml.safe_dump({'graspgenx_server': values}))
    return path


@pytest.fixture
def server(tmp_path: Path) -> GraspServer:
    config = ServerConfig.from_yaml(write_config(tmp_path))
    model = GraspModel(config)
    model.sampler = object()
    return GraspServer(config, model)


def request(points: np.ndarray, num_grasps: int = 0) -> list[bytes]:
    return [HEADER.pack(PROTOCOL_VERSION, num_grasps), points.astype('<f4').tobytes()]


def read_reply(frames: list[bytes]) -> tuple[bool, str, np.ndarray]:
    assert len(frames) == 3
    version, success = HEADER.unpack(frames[0])
    assert version == PROTOCOL_VERSION
    return bool(success), frames[1].decode(), np.frombuffer(frames[2], '<f8').reshape(-1, 10)


def lifted(z_m: float) -> np.ndarray:
    pose = np.eye(4)
    pose[2, 3] = z_m
    return pose


def block(n: int) -> np.ndarray:
    """A 6 x 2 x 3 cm block between the open fingers of a grasp at the origin."""
    return np.random.default_rng(1).uniform([-0.03, -0.01, 0.11], [0.03, 0.01, 0.14], (n, 3))


def test_too_few_points_are_refused(server: GraspServer) -> None:
    points = np.vstack([block(10), np.full((60, 3), np.nan)])
    success, message, rows = read_reply(server.answer(request(points)))
    assert not success and 'at least 50' in message and len(rows) == 0


def test_more_grasps_than_max_grasps_are_refused(server: GraspServer) -> None:
    success, message, _ = read_reply(server.answer(request(block(60), num_grasps=3)))
    assert not success and 'max_grasps' in message


def test_grasps_come_back_best_first_capped_and_as_tcp_poses(server: GraspServer) -> None:
    seen = {}

    def planner(points: np.ndarray, sampler: object, **kwargs: Any) -> tuple:
        seen['points'] = len(points)
        seen['num_grasps'] = kwargs['num_grasps']
        return np.stack([lifted(0.0), lifted(0.01), lifted(0.02)]), np.array([0.2, 0.9, 0.5]), []

    server.model.run_planner = planner
    success, message, rows = read_reply(server.answer(request(block(300))))
    assert success, message
    assert rows.shape == (2, REPLY_COLUMNS)
    np.testing.assert_allclose(rows[:, 7], [0.9, 0.5])
    # Best first: the grasp 1 cm up, its TCP 0.1295 m along the approach.
    np.testing.assert_allclose(rows[0, 0:3], [0.0, 0.0, 0.1395], atol=1e-9)
    # The TCP's y closes along GraspGenX's x: a quarter turn about z.
    np.testing.assert_allclose(np.abs(rows[0, 3:7]), [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    assert seen == {'points': 100, 'num_grasps': 200}
    assert 0.0 < rows[0, 9] <= rows[0, 8] <= 0.07 + 2 * 0.008


def test_a_failing_model_is_answered_and_a_cuda_error_ends_the_server(server: GraspServer) -> None:
    def planner(*args: object, **kwargs: object) -> tuple:
        raise RuntimeError('CUDA error: an illegal memory access was encountered')

    server.model.run_planner = planner
    success, message, _ = read_reply(server.answer(request(block(60))))
    assert not success and 'illegal memory access' in message
    assert server.failed_fatally


@pytest.mark.parametrize(
    'frames',
    [
        [b'x'],
        [b'short', b''],
        [HEADER.pack(PROTOCOL_VERSION + 1, 0), b''],
        [HEADER.pack(PROTOCOL_VERSION, 0), b'\x00' * 13],
    ],
)
def test_a_malformed_request_is_answered(server: GraspServer, frames: list[bytes]) -> None:
    success, message, _ = read_reply(server.answer(frames))
    assert not success and message.startswith('Bad request')
    assert not server.failed_fatally


@pytest.mark.parametrize(
    ('overrides', 'match'),
    [
        ({'offset_open_m': [0.01, 0.0, 0.12]}, 'centred'),
        ({'extents_mid_m': [0.03, 0.02]}, 'three numbers'),
        ({'planner': 'moe'}, 'planner'),
        ({'max_points': 10}, 'min_points'),
        ({'typo_m': 1.0}, 'unknown'),
    ],
)
def test_bad_parameters_are_refused(tmp_path: Path, overrides: dict, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        ServerConfig.from_yaml(write_config(tmp_path, **overrides))


def test_a_missing_checkpoint_is_refused(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match='checkpoint_dir'):
        ServerConfig.from_yaml(write_config(tmp_path, checkpoint_dir=str(tmp_path / 'none')))


def test_the_parameter_file_agrees_with_the_node_and_the_robot_description() -> None:
    server = yaml.safe_load((PACKAGE / 'config/graspgenx_server.yaml').read_text())
    server = server['graspgenx_server']
    assert set(server) == set(BASE) | {'checkpoint_dir'}
    node = yaml.safe_load((PACKAGE / 'config/grasp_generation.yaml').read_text())
    assert node['/**/grasp_generation']['ros__parameters']['server_endpoint'] == server['endpoint']
    xacro = (PACKAGE.parent / 'piper_description/urdf/piper_arm.xacro').read_text()
    tcp = re.search(r'tcp_joint".*?<origin xyz="0 0 ([0-9.]+)"', xacro, re.DOTALL)
    assert tcp is not None and float(tcp.group(1)) == server['tcp_depth_m']

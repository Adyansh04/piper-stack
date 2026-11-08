"""Frame conversions between GraspGenX and piper_msgs/Grasp."""

from __future__ import annotations

import numpy as np
import pytest
from frames import measure_finger_extents, to_tcp_poses
from scipy.spatial.transform import Rotation

EXTENTS_M = np.array([0.177, 0.033, 0.09])
OFFSET_M = np.array([0.0, 0.0, 0.155])
# A 6 cm block 2 cm off centre and a point beyond the fingers, in the gripper frame.
BLOCK_M = np.array([[-0.01, 0.0, 0.15], [0.05, 0.0, 0.15], [0.02, 0.01, 0.16], [0.5, 0.0, 0.15]])


def measure(points: np.ndarray, poses: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return measure_finger_extents(
        points, poses, extents_open_m=EXTENTS_M, offset_open_m=OFFSET_M, finger_thickness_m=0.02
    )


def rigid(seed: int) -> np.ndarray:
    """A random rigid transform [4, 4]."""
    rng = np.random.default_rng(seed)
    out = np.eye(4)
    out[:3, :3] = Rotation.random(random_state=seed).as_matrix()
    out[:3, 3] = rng.uniform(-1.0, 1.0, 3)
    return out


def test_tcp_sits_at_the_fingertips_with_z_approach_and_y_closing() -> None:
    tcp = to_tcp_poses(np.eye(4)[None], 0.2)[0]
    np.testing.assert_allclose(tcp[:3, 3], [0.0, 0.0, 0.2], atol=1e-12)
    np.testing.assert_allclose(tcp[:3, 2], [0.0, 0.0, 1.0], atol=1e-12)  # approach
    np.testing.assert_allclose(tcp[:3, 1], [1.0, 0.0, 0.0], atol=1e-12)  # closing: GraspGenX +X
    np.testing.assert_allclose(np.linalg.det(tcp[:3, :3]), 1.0, atol=1e-12)


@pytest.mark.parametrize('seed', range(5))
def test_tcp_of_a_moved_gripper_moves_with_it(seed: int) -> None:
    pose = rigid(seed)
    tcp = to_tcp_poses(pose[None], 0.2)[0]
    np.testing.assert_allclose(tcp[:3, 3], pose[:3, 3] + 0.2 * pose[:3, 2], atol=1e-12)
    np.testing.assert_allclose(tcp[:3, 2], pose[:3, 2], atol=1e-12)
    np.testing.assert_allclose(tcp[:3, 1], pose[:3, 0], atol=1e-12)


def test_opening_counts_the_farthest_point_and_width_the_object_between_the_fingers() -> None:
    opening, width = measure(BLOCK_M, np.eye(4)[None])
    np.testing.assert_allclose(opening, [0.10], atol=1e-12)
    np.testing.assert_allclose(width, [0.06], atol=1e-12)
    # A point in a finger, 9.5 cm out, counts towards both.
    opening, width = measure(np.vstack([BLOCK_M, [[-0.095, 0.0, 0.17]]]), np.eye(4)[None])
    np.testing.assert_allclose(opening, [0.19], atol=1e-12)
    np.testing.assert_allclose(width, [0.145], atol=1e-12)


@pytest.mark.parametrize('seed', range(5))
def test_extents_do_not_depend_on_where_the_scene_is(seed: int) -> None:
    move = rigid(seed)
    points = BLOCK_M @ move[:3, :3].T + move[:3, 3]
    opening, width = measure(points, move[None])
    np.testing.assert_allclose(opening, [0.10], atol=1e-9)
    np.testing.assert_allclose(width, [0.06], atol=1e-9)


def test_no_points_no_poses_and_non_finite_points() -> None:
    opening, width = measure(np.zeros((0, 3)), np.eye(4)[None])
    np.testing.assert_array_equal(opening, [0.0])
    np.testing.assert_array_equal(width, [0.0])
    opening, width = measure(BLOCK_M, np.zeros((0, 4, 4)))
    assert opening.shape == (0,) and width.shape == (0,)
    with_nan = np.vstack([BLOCK_M, [[np.nan, 0.0, 0.15], [np.inf, 0.0, 0.15]]])
    opening, _ = measure(with_nan, np.eye(4)[None])
    np.testing.assert_allclose(opening, [0.10], atol=1e-12)


def test_points_outside_the_slab_give_zero() -> None:
    far = np.array([[0.0, 0.2, 0.15], [0.0, 0.0, 0.5]])
    opening, width = measure(far, np.eye(4)[None])
    np.testing.assert_array_equal(opening, [0.0])
    np.testing.assert_array_equal(width, [0.0])

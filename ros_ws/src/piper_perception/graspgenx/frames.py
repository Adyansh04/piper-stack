"""Frame conventions between GraspGenX and piper_msgs/Grasp, in numpy (no torch).

GraspGenX poses the gripper's base frame: +Z is the approach direction, +X the closing direction,
the fingertips at `fingertip_depth_m` along +Z. A `Grasp` poses the tool centre point: z along the
approach, y between the fingers (the closing direction), x = y cross z.
"""

from __future__ import annotations

import numpy as np

# Columns: the TCP's x, y, z axes in GraspGenX's gripper frame.
_TCP_IN_GRIPPER = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def to_tcp_poses(gripper_poses: np.ndarray, fingertip_depth_m: float) -> np.ndarray:
    """Return the TCP poses of GraspGenX gripper-base poses.

    Args:
        gripper_poses: Gripper-base poses [N, 4, 4].
        fingertip_depth_m: Gripper base to the TCP along the approach, metres.

    Returns:
        The TCP poses [N, 4, 4], in the same parent frame.
    """
    out = np.array(gripper_poses, dtype=np.float64, copy=True)
    out[:, :3, 3] += out[:, :3, 2] * fingertip_depth_m
    out[:, :3, :3] = out[:, :3, :3] @ _TCP_IN_GRIPPER
    return out


def measure_finger_extents(
    points: np.ndarray,
    gripper_poses: np.ndarray,
    *,
    extents_open_m: np.ndarray,
    offset_open_m: np.ndarray,
    finger_thickness_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Measure what the fingers meet per grasp, from the points in their slab to their outer faces.

    Args:
        points: Object points [M, 3], the poses' parent frame, metres; non-finite ones are ignored.
        gripper_poses: GraspGenX gripper-base poses [N, 4, 4].
        extents_open_m: Box between the open fingertips (closing, finger width, finger length),
            metres.
        offset_open_m: Centre of that box from the gripper base, metres; 0 along the closing axis,
            where the fingers' centre plane is.
        finger_thickness_m: The fingers' own thickness along the closing axis, metres.

    Returns:
        The opening each grasp needs (twice the farthest point from the fingers' centre plane, so an
        object off centre needs more) and the object's width between the fingers, metres; 0 and 0
        when no point is in the slab.
    """
    half = np.asarray(extents_open_m, dtype=np.float64) / 2.0
    offset = np.asarray(offset_open_m, dtype=np.float64)
    reach = half[0] + finger_thickness_m
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    # Rows x, y, z, contiguous: each grasp's slab test runs on whole rows.
    points_t = np.ascontiguousarray(points.T)
    opening = np.zeros(len(gripper_poses))
    width = np.zeros(len(gripper_poses))
    for i, pose in enumerate(gripper_poses):
        local = pose[:3, :3].T @ (points_t - pose[:3, 3:4])
        inside = (
            (np.abs(local[0] - offset[0]) <= reach)
            & (np.abs(local[1] - offset[1]) <= half[1])
            & (np.abs(local[2] - offset[2]) <= half[2])
        )
        if inside.any():
            x = local[0, inside]
            opening[i] = 2.0 * np.abs(x).max()
            width[i] = x.max() - x.min()
    return opening, width

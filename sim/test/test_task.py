"""The task specs under tasks/ and their loader (no Isaac Sim needed)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import task

TASKS = Path(__file__).resolve().parents[2] / 'tasks'
#: Top-down reach of the arm (sim.yaml): the gripper pointing down reaches about this far.
TOP_DOWN_REACH_M = 0.33


def write_cube_stl(path: Path, size_mm: float) -> None:
    """Write a binary STL of a cube with one corner at the origin, outward-facing."""
    corners = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)]) * size_mm
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    triangles = [(a, b, c) for a, b, c, d in faces] + [(a, c, d) for a, b, c, d in faces]
    data = np.zeros(len(triangles), dtype=task.STL_TRIANGLE)
    data['vertices'] = corners[np.array(triangles)]
    path.write_bytes(bytes(80) + len(triangles).to_bytes(4, 'little') + data.tobytes())


@pytest.fixture(scope='module')
def tasks() -> dict[str, task.Task]:
    return {d.name: task.load(d) for d in sorted(TASKS.iterdir()) if (d / 'task.yaml').is_file()}


def test_every_task_loads_with_its_meshes_and_poses(tasks: dict[str, task.Task]) -> None:
    assert {'assembly', 'usb_c'} <= tasks.keys()
    for t in tasks.values():
        for part in t.parts.values():
            assert part.shapes and all(len(s.triangles) for s in part.shapes)
            assert (part.mass_kg > 0.0) == part.dynamic


def test_a_printed_mass_follows_the_cad_volume(tasks: dict[str, task.Task]) -> None:
    pin = tasks['assembly'].parts['pin']
    # The pin's CAD volume is 1585.25 mm3 (Ø8 x 32 with 1 mm chamfers), printed half full.
    assert task.mesh_volume_m3(pin.shapes[0].triangles) == pytest.approx(1585.25e-9, rel=1e-3)
    assert pin.mass_kg == pytest.approx(1585.25e-9 * 1240.0 * 0.5, rel=1e-3)


def test_loose_parts_start_within_the_top_down_reach(tasks: dict[str, task.Task]) -> None:
    for t in tasks.values():
        for name, T in t.states['start'].items():
            assert np.hypot(*T[:2, 3]) < TOP_DOWN_REACH_M, f'{t.name}.{name}'


def test_goal_poses_are_in_the_frames_they_name(tasks: dict[str, task.Task]) -> None:
    assembly = tasks['assembly']
    base = assembly.states['goal']['base']
    np.testing.assert_allclose(base[:3, 3], [0.214, -0.025, 0.005], atol=1e-12)
    ring = assembly.states['goal']['ring']
    # The fixture turns the base a quarter turn: the post at -x of the base lies at -y.
    np.testing.assert_allclose(ring[:3, 3], [0.214, -0.06, 0.017], atol=1e-12)
    np.testing.assert_allclose(ring[:3, 0], [0.0, -1.0, 0.0], atol=1e-12)


def test_the_plug_points_into_the_socket(tasks: dict[str, task.Task]) -> None:
    usb_c = tasks['usb_c']
    socket = usb_c.fixed['socket']
    start, goal = usb_c.states['start']['plug'], usb_c.states['goal']['plug']
    # The socket opens towards the arm's side (-y) with its long side vertical.
    np.testing.assert_allclose(socket[:3, 2], [0.0, -1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(socket[:3, 0], [0.0, 0.0, 1.0], atol=1e-12)
    # The plug waits pointing at it, at its height, and ends 6.6 mm deep.
    np.testing.assert_allclose(start[:3, 2], [0.0, 1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(start[:3, 0], socket[:3, 0], atol=1e-12)
    assert start[2, 3] == pytest.approx(socket[2, 3])
    np.testing.assert_allclose(goal[:3, 3], socket[:3, 3] + 0.0066 * np.array([0, 1, 0]))


def test_the_cable_segments_join_from_plug_to_anchor(tasks: dict[str, task.Task]) -> None:
    usb_c = tasks['usb_c']
    cable = usb_c.cable
    assert cable is not None
    half = cable.rotations[:, :, 0] * cable.lengths_m[:, None] / 2.0
    starts, ends = cable.centres - half, cable.centres + half
    np.testing.assert_allclose(ends[:-1], starts[1:], atol=1e-12)
    plug = usb_c.states['start'][cable.plug]
    np.testing.assert_allclose(starts[0], (plug @ np.append(cable.attach, 1.0))[:3], atol=1e-12)
    np.testing.assert_allclose(
        cable.rotations @ cable.rotations.transpose(0, 2, 1),
        np.eye(3)[None].repeat(len(half), 0),
        atol=1e-9,
    )
    assert np.all(cable.lengths_m <= 0.01 + 1e-12)


def test_a_planar_cable_starts_without_twist() -> None:
    angles = np.linspace(0.0, np.pi, 20)
    arc = np.column_stack((np.cos(angles), np.sin(angles), np.zeros_like(angles))) * 0.1
    _, rotations, _ = task.cable_frames(arc, 0.01)
    normals = rotations.transpose(0, 2, 1) @ np.array([0.0, 0.0, 1.0])
    np.testing.assert_allclose(normals, np.repeat(normals[:1], len(normals), 0), atol=1e-9)


@pytest.mark.parametrize(
    'points', [[[0, 0, 0]], [[0, 0, 0], [0, 0, 0], [0, 0, 1]], [[0, 0, 0], [1, 0, 0], [0, 0, 0]]]
)
def test_a_cable_path_with_too_few_points_a_repeat_or_a_reversal_is_refused(
    points: list[list[float]],
) -> None:
    with pytest.raises(task.TaskError):
        task.cable_frames(np.asarray(points, dtype=float), 0.01)


def test_a_cube_has_the_inertia_of_a_cube(tmp_path: Path) -> None:
    write_cube_stl(tmp_path / 'cube.stl', 20.0)
    com, moments, axes = task.mass_properties(task.stl_triangles(tmp_path / 'cube.stl'), 0.5)
    np.testing.assert_allclose(com, [0.01, 0.01, 0.01], atol=1e-12)
    # A solid cube: m s^2 / 6 about every axis through its centre.
    np.testing.assert_allclose(moments, [0.5 * 0.02**2 / 6.0] * 3, rtol=1e-9)
    assert np.linalg.det(axes) == pytest.approx(1.0)


def test_the_pin_spins_easier_than_it_tumbles(tasks: dict[str, task.Task]) -> None:
    pin = tasks['assembly'].parts['pin']
    # A slender cylinder: least inertia about its axis (z), the centre half-way up.
    np.testing.assert_allclose(pin.com_m, [0.0, 0.0, 0.016], atol=1e-5)
    np.testing.assert_allclose(np.abs(pin.principal_axes[:, 0]), [0.0, 0.0, 1.0], atol=1e-6)
    assert pin.inertia_kg_m2[0] < pin.inertia_kg_m2[1] / 10


def test_a_pose_in_an_unknown_frame_is_refused(tmp_path: Path) -> None:
    write_cube_stl(tmp_path / 'cube.stl', 10.0)
    assert task.mesh_volume_m3(task.stl_triangles(tmp_path / 'cube.stl')) == pytest.approx(1e-6)
    (tmp_path / 'task.yaml').write_text(
        'name: t\n'
        'materials: {pla: {static_friction: 0.3, dynamic_friction: 0.3, restitution: 0.0,'
        ' contact_stiffness: 1.0e5, contact_damping: 0.0, torsional_friction_m: 0.0,'
        ' rolling_friction_m: 0.0}}\n'
        'contact: {gap_m: 0.001, sdf_band_m: 0.001}\n'
        'print: {density_kg_m3: 1000.0, fill: 1.0}\n'
        'parts:\n'
        '  cube: {body: dynamic, material: pla,'
        ' shapes: [{mesh: cube.stl, collision: convex_hull, colour: [1, 1, 1]}]}\n'
        'scene: {start: {cube: {in: nowhere, xyz: [0, 0, 0]}}}\n'
    )
    with pytest.raises(task.TaskError, match='nowhere'):
        task.load(tmp_path)


def test_the_assembly_stages_come_in_order(tasks: dict[str, task.Task]) -> None:
    assembly = tasks['assembly']
    assert [name for name, _ in assembly.stages] == ['ring', 'collar', 'pin', 'cap']
    before_ring = task.state_before(assembly, 'ring')
    for name, T in before_ring.items():
        np.testing.assert_allclose(T, assembly.states['start'][name])
    before_cap = task.state_before(assembly, 'cap')
    for name in ('ring', 'collar', 'pin'):
        np.testing.assert_allclose(before_cap[name], assembly.states['goal'][name])
    np.testing.assert_allclose(before_cap['cap'], assembly.states['start']['cap'])
    with pytest.raises(task.TaskError, match='no stage'):
        task.state_before(assembly, 'nut')


def test_a_moved_scene_keeps_every_part_where_it_is_on_the_fixture(
    tasks: dict[str, task.Task],
) -> None:
    assembly = tasks['assembly']
    move = task.placement(assembly, np.random.default_rng(3), xy_m=0.015, yaw_deg=8.0)
    shifted = task.moved(assembly, move)
    fixture, moved_fixture = assembly.fixed['fixture'], shifted.fixed['fixture']
    # The fixture moves within the bounds about its own centre, staying flat.
    assert np.all(np.abs(moved_fixture[:2, 3] - fixture[:2, 3]) <= 0.015 + 1e-12)
    assert abs(np.degrees(np.arctan2(move[1, 0], move[0, 0]))) <= 8.0
    np.testing.assert_allclose(moved_fixture[2], fixture[2], atol=1e-12)
    for state in ('start', 'goal'):
        for name, T in assembly.states[state].items():
            np.testing.assert_allclose(
                np.linalg.inv(moved_fixture) @ shifted.states[state][name],
                np.linalg.inv(fixture) @ T,
                atol=1e-12,
            )

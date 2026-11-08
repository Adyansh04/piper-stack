"""A task's parts, poses and cable, read from ``tasks/<name>/task.yaml`` and checked on load.

NumPy and the standard library only, so the simulator, the preview and the tests read one spec
without Isaac Sim. Poses are 4 x 4 transforms in the robot's world frame (its base at the origin on
the table top), in metres; the CAD meshes are in millimetres.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

COLLISIONS = ('sdf', 'convex_hull', 'convex_decomposition', 'none')
BODIES = ('static', 'dynamic')
STATES = ('start', 'goal')
#: A binary STL triangle: normal, three vertices, attribute byte count.
STL_TRIANGLE = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attributes', '<u2')])
#: CAD meshes are in millimetres.
MESH_SCALE_M = 1e-3


class TaskError(ValueError):
    """A task.yaml that cannot be used; the message names the file and the entry."""


@dataclass(frozen=True, slots=True)
class Material:
    """A surface's contact properties.

    ``contact_stiffness`` (N/m) and ``contact_damping`` (N s/m) set how hard Newton's contacts are;
    ``torsional_friction_m`` and ``rolling_friction_m`` are the lever arms (m) of friction against
    spinning and rolling about the contact normal.
    """

    static_friction: float
    dynamic_friction: float
    restitution: float
    contact_stiffness: float
    contact_damping: float
    torsional_friction_m: float
    rolling_friction_m: float


@dataclass(frozen=True, slots=True)
class Contact:
    """Where contacts are looked for: within ``gap_m`` of a surface, and a mesh's signed distance
    field stored within ``sdf_band_m`` of it on either side (metres)."""

    gap_m: float
    sdf_band_m: float


@dataclass(frozen=True, slots=True)
class Shape:
    """One mesh of a part, with its collision and look.

    ``triangles`` (n x 3 x 3) are in metres, in the part's frame; ``sdf_resolution`` counts the
    signed distance field's cells along the mesh's longest side (sdf collision only).
    """

    mesh: Path
    triangles: np.ndarray
    collision: str
    sdf_resolution: int
    colour: tuple[float, float, float]
    metallic: float
    roughness: float


@dataclass(frozen=True, slots=True)
class Part:
    """A part: static (fixed in the world) or dynamic (a rigid body of ``mass_kg``).

    A dynamic part's ``com_m`` and principal ``inertia_kg_m2`` (about the centre of mass, along the
    columns of ``principal_axes``) follow from its meshes at a uniform density; static parts have
    zeros.
    """

    name: str
    dynamic: bool
    material: str
    mass_kg: float
    com_m: np.ndarray
    inertia_kg_m2: np.ndarray
    principal_axes: np.ndarray
    shapes: tuple[Shape, ...]


@dataclass(frozen=True, slots=True)
class Cable:
    """A cable as a chain of segments from its plug to its anchor, at its start state.

    ``attach`` is where the cable leaves the plug, in the plug's frame. The segments' ``centres``
    (n x 3) and ``rotations`` (n x 3 x 3, x along the cable away from the plug) are in the world
    frame; ``lengths_m`` are their lengths. The stiffnesses are the cable's bending and torsional
    rigidity (EI and GJ, N m2); the damping is per joint (N m s/rad).
    """

    plug: str
    attach: np.ndarray
    colour: tuple[float, float, float]
    radius_m: float
    mass_kg_per_m: float
    bend_stiffness_n_m2: float
    twist_stiffness_n_m2: float
    damping_n_m_s: float
    centres: np.ndarray
    rotations: np.ndarray
    lengths_m: np.ndarray


@dataclass(frozen=True, slots=True)
class Task:
    """A task: its parts, where they are and its cable.

    ``fixed`` holds the static parts' world poses; ``states`` the dynamic parts' world poses in
    each state (start, goal); ``stages`` each stage's name and part, in order; ``preview`` is
    task.yaml's preview section as written.
    """

    name: str
    root: Path
    materials: dict[str, Material]
    contact: Contact
    parts: dict[str, Part]
    fixed: dict[str, np.ndarray]
    states: dict[str, dict[str, np.ndarray]]
    cable: Cable | None
    preview: dict
    stages: tuple[tuple[str, str], ...] = ()


def pose_matrix(xyz: list[float], rpy_deg: list[float]) -> np.ndarray:
    """Return the 4 x 4 transform of a translation and fixed-axis roll, pitch, yaw (as in URDF).

    Args:
        xyz: Translation, metres.
        rpy_deg: Rotations about the fixed x, then y, then z axes, degrees.
    """
    roll, pitch, yaw = np.radians(np.asarray(rpy_deg, dtype=float))
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def stl_triangles(path: Path) -> np.ndarray:
    """Return a binary STL's triangles (n x 3 x 3), scaled from millimetres to metres.

    Raises:
        TaskError: If the file is not a binary STL.
    """
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], 'little') if len(data) >= 84 else -1
    if count < 0 or len(data) != 84 + count * STL_TRIANGLE.itemsize:
        raise TaskError(f'{path}: not a binary STL')
    triangles = np.frombuffer(data, dtype=STL_TRIANGLE, count=count, offset=84)['vertices']
    return triangles.astype(np.float64) * MESH_SCALE_M


def mesh_volume_m3(triangles: np.ndarray) -> float:
    """Return the volume a closed, outward-facing triangle mesh encloses, cubic metres."""
    v0, v1, v2 = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    return float(np.einsum('ij,ij->', v0, np.cross(v1, v2)) / 6.0)


def mass_properties(triangles: np.ndarray, mass_kg: float) -> tuple[np.ndarray, ...]:
    """Return a closed mesh's centre of mass and principal inertia at a uniform density.

    Args:
        triangles: The mesh's outward-facing triangles (n x 3 x 3), metres.
        mass_kg: The mass the mesh's volume holds.

    Returns:
        The centre of mass (3), the principal moments of inertia about it (3, kg m2) and the
        principal axes (3 x 3, columns, a right-handed rotation).

    Raises:
        TaskError: If the mesh encloses no volume.
    """
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    # Tetrahedra from the origin: six times their signed volumes.
    det = np.einsum('ij,ij->i', a, np.cross(b, c))
    volume = det.sum() / 6.0
    if volume <= 0.0:
        raise TaskError('A mesh encloses no volume: is it closed and facing out?')
    com = (det[:, None] * (a + b + c)).sum(0) / 24.0 / volume
    s = a + b + c
    second = np.einsum('i,ij,ik->jk', det, a, a) + np.einsum('i,ij,ik->jk', det, b, b)
    second += np.einsum('i,ij,ik->jk', det, c, c) + np.einsum('i,ij,ik->jk', det, s, s)
    covariance = second / 120.0 - volume * np.outer(com, com)
    inertia = (np.trace(covariance) * np.eye(3) - covariance) * (mass_kg / volume)
    moments, axes = np.linalg.eigh(inertia)
    if np.linalg.det(axes) < 0.0:
        axes[:, 2] *= -1.0
    return com, moments, axes


def _least_rotation(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return the smallest rotation that turns unit vector a onto unit vector b."""
    v, c = np.cross(a, b), float(a @ b)
    if c < -0.99:
        raise TaskError('The cable path turns back on itself')
    K = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + K + K @ K / (1.0 + c)


def cable_frames(points: np.ndarray, segment_length_m: float) -> tuple[np.ndarray, ...]:
    """Cut a path into segments and give each an untwisted frame.

    The path is split at equal arc lengths into the fewest segments no longer than
    ``segment_length_m``; each segment is the chord between two split points. Its frame has x
    along the chord, away from the path's first point, and is the previous segment's frame turned
    by the least rotation onto it, so the chain starts without twist.

    Args:
        points: The path's points (n x 3), metres; the first is the plug's end.
        segment_length_m: The longest a segment may be, metres.

    Returns:
        The segments' centres (m x 3), rotations (m x 3 x 3, columns the frame's axes) and
        lengths (m), metres.

    Raises:
        TaskError: If the path has fewer than two points, a step of zero length or turns back.
    """
    if len(points) < 2 or segment_length_m <= 0.0:
        raise TaskError('A cable path needs two points or more and a positive segment length')
    step_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(step_lengths < 1e-6):
        raise TaskError('The cable path repeats a point')
    arc = np.concatenate(([0.0], np.cumsum(step_lengths)))
    count = int(np.ceil(arc[-1] / segment_length_m - 1e-9))
    s = np.linspace(0.0, arc[-1], count + 1)
    split = np.column_stack([np.interp(s, arc, points[:, k]) for k in range(3)])
    chords = np.diff(split, axis=0)
    lengths = np.linalg.norm(chords, axis=1)
    x = chords / lengths[:, None]
    helper = np.eye(3)[np.argmin(np.abs(x[0]))]
    y0 = np.cross(helper, x[0])
    y0 /= np.linalg.norm(y0)
    rotations = np.empty((count, 3, 3))
    rotations[0] = np.column_stack((x[0], y0, np.cross(x[0], y0)))
    for i in range(1, count):
        rotations[i] = _least_rotation(x[i - 1], x[i]) @ rotations[i - 1]
    return (split[:-1] + split[1:]) / 2.0, rotations, lengths


def _require(mapping: dict, key: str, where: str):
    """Return mapping[key], or raise a TaskError that names where it is missing."""
    if not isinstance(mapping, dict) or key not in mapping:
        raise TaskError(f'{where}: missing {key!r}')
    return mapping[key]


def _resolve(entries: dict, known: dict[str, np.ndarray], where: str) -> dict[str, np.ndarray]:
    """Return the world pose of every entry, following ``in`` through known poses and the entries.

    Raises:
        TaskError: If a pose is in a frame no entry or known pose gives, or poses form a loop.
    """
    resolved = dict(known)

    def pose_of(name: str, chain: tuple[str, ...]) -> np.ndarray:
        if name in resolved:
            return resolved[name]
        if name not in entries:
            raise TaskError(f'{where}: no pose for {name!r}')
        if name in chain:
            raise TaskError(f'{where}: poses refer to each other in a loop: {chain + (name,)}')
        entry = entries[name]
        local = pose_matrix(entry.get('xyz', [0.0, 0.0, 0.0]), entry.get('rpy', [0.0, 0.0, 0.0]))
        frame = entry.get('in')
        resolved[name] = local if frame is None else pose_of(frame, chain + (name,)) @ local
        return resolved[name]

    for name in entries:
        pose_of(name, ())
    return {name: resolved[name] for name in entries}


def _colour(value: list, where: str) -> tuple[float, float, float]:
    """Return a colour's red, green and blue, each 0 to 1."""
    rgb = tuple(float(c) for c in value)
    if len(rgb) != 3 or not all(0.0 <= c <= 1.0 for c in rgb):
        raise TaskError(f'{where}: a colour is red, green and blue, each 0 to 1')
    return rgb[0], rgb[1], rgb[2]


def _shape(spec: dict, root: Path, where: str) -> Shape:
    """Return a part's shape from its spec, with the mesh read."""
    mesh = root / _require(spec, 'mesh', where)
    collision = _require(spec, 'collision', where)
    if collision not in COLLISIONS:
        raise TaskError(f'{where}: collision {collision!r} is not one of {COLLISIONS}')
    if not mesh.is_file():
        raise TaskError(f'{where}: no mesh at {mesh}')
    return Shape(
        mesh=mesh,
        triangles=stl_triangles(mesh),
        collision=collision,
        sdf_resolution=int(spec.get('sdf_resolution', 256)),
        colour=_colour(_require(spec, 'colour', where), where),
        metallic=float(spec.get('metallic', 0.0)),
        roughness=float(spec.get('roughness', 0.5)),
    )


def _part(name: str, spec: dict, root: Path, printing: dict | None, where: str) -> Part:
    """Return a part from its spec; a dynamic part without mass_kg gets its printed mass."""
    body = _require(spec, 'body', where)
    if body not in BODIES:
        raise TaskError(f'{where}: body {body!r} is not one of {BODIES}')
    shapes = tuple(
        _shape(s, root, f'{where}.shapes[{i}]')
        for i, s in enumerate(_require(spec, 'shapes', where))
    )
    mass_kg = 0.0
    if body == 'dynamic':
        if 'mass_kg' in spec:
            mass_kg = float(spec['mass_kg'])
        else:
            density = float(_require(printing, 'density_kg_m3', 'print'))
            volume_m3 = sum(mesh_volume_m3(s.triangles) for s in shapes)
            mass_kg = volume_m3 * density * float(_require(printing, 'fill', 'print'))
        if mass_kg <= 0.0:
            raise TaskError(f'{where}: a dynamic part needs a positive mass')
        com, moments, axes = mass_properties(np.concatenate([s.triangles for s in shapes]), mass_kg)
    else:
        com, moments, axes = np.zeros(3), np.zeros(3), np.eye(3)
    return Part(
        name,
        body == 'dynamic',
        _require(spec, 'material', where),
        mass_kg,
        com,
        moments,
        axes,
        shapes,
    )


def _cable(spec: dict, start: dict[str, np.ndarray], fixed: dict, where: str) -> Cable:
    """Return the cable from its spec: the path runs from the plug's start pose to the anchor."""
    plug = _require(spec, 'plug', where)
    if plug not in start:
        raise TaskError(f'{where}: the plug {plug!r} has no start pose')
    attach = np.asarray(_require(spec, 'attach', where), dtype=float)
    path = _require(spec, 'path', where)
    frame = path.get('in')
    if frame is not None and frame not in fixed:
        raise TaskError(f'{where}.path: {frame!r} is not a static part')
    T = np.eye(4) if frame is None else fixed[frame]
    points = np.asarray(_require(path, 'points', f'{where}.path'), dtype=float)
    plug_end = (start[plug] @ np.append(attach, 1.0))[:3]
    world = np.vstack((plug_end, points @ T[:3, :3].T + T[:3, 3]))
    centres, rotations, lengths = cable_frames(
        world, float(_require(spec, 'segment_length_m', where))
    )
    keys = (
        'radius_m',
        'mass_kg_per_m',
        'bend_stiffness_n_m2',
        'twist_stiffness_n_m2',
        'damping_n_m_s',
    )
    values = {key: float(_require(spec, key, where)) for key in keys}
    if min(values.values()) < 0.0 or values['radius_m'] == 0.0 or values['mass_kg_per_m'] == 0.0:
        raise TaskError(f'{where}: radius and mass must be positive, the rest not negative')
    return Cable(
        plug,
        attach,
        _colour(_require(spec, 'colour', where), where),
        **values,
        centres=centres,
        rotations=rotations,
        lengths_m=lengths,
    )


def load(task_dir: Path) -> Task:
    """Read and check a task's spec, its meshes and its poses.

    Args:
        task_dir: The task's directory, holding task.yaml; mesh paths are relative to it.

    Raises:
        TaskError: If the spec is incomplete, names an unknown part, material or frame, or a mesh
            is missing or not a binary STL.
    """
    path = task_dir / 'task.yaml'
    spec = yaml.safe_load(path.read_text())
    where = str(path)
    materials = {
        name: Material(
            **{
                key: float(_require(m, key, f'{where}: materials.{name}'))
                for key in Material.__dataclass_fields__
            }
        )
        for name, m in _require(spec, 'materials', where).items()
    }
    contact_spec = _require(spec, 'contact', where)
    contact = Contact(
        float(_require(contact_spec, 'gap_m', f'{where}: contact')),
        float(_require(contact_spec, 'sdf_band_m', f'{where}: contact')),
    )
    parts = {
        name: _part(name, p, task_dir, spec.get('print'), f'{where}: parts.{name}')
        for name, p in _require(spec, 'parts', where).items()
    }
    for part in parts.values():
        if part.material not in materials:
            raise TaskError(f'{where}: parts.{part.name}: no material {part.material!r}')
    scene = _require(spec, 'scene', where)
    fixed = _resolve(scene.get('fixed', {}), {}, f'{where}: scene.fixed')
    start = _resolve(_require(scene, 'start', f'{where}: scene'), fixed, f'{where}: scene.start')
    goal_entries = scene.get('goal', {})
    unmoved = {n: T for n, T in start.items() if n not in goal_entries}
    goal = {**start, **_resolve(goal_entries, {**fixed, **unmoved}, f'{where}: scene.goal')}
    for name in fixed:
        if name not in parts or parts[name].dynamic:
            raise TaskError(f'{where}: scene.fixed places {name!r}, which is no static part')
    for name in goal:
        if name not in parts or not parts[name].dynamic:
            raise TaskError(f'{where}: scene.start or goal places {name!r}, no dynamic part')
    for name, part in parts.items():
        if name not in (start if part.dynamic else fixed):
            kind = 'start' if part.dynamic else 'fixed'
            raise TaskError(f'{where}: parts.{name} has no pose in scene.{kind}')
    cable = None
    if 'cable' in spec:
        cable = _cable(spec['cable'], start, fixed, f'{where}: cable')
        if not parts[cable.plug].dynamic:
            raise TaskError(f'{where}: cable.plug {cable.plug!r} must be a dynamic part')
    return Task(
        name=_require(spec, 'name', where),
        root=task_dir,
        materials=materials,
        contact=contact,
        parts=parts,
        fixed=fixed,
        states={'start': start, 'goal': goal},
        cable=cable,
        preview=spec.get('preview', {}),
        stages=_stages(spec.get('stages', []), parts, goal, f'{where}: stages'),
    )


def _stages(entries: list, parts: dict, goal: dict, where: str) -> tuple[tuple[str, str], ...]:
    """Return each stage's name and part; a stage's part must be dynamic with a goal pose.

    Raises:
        TaskError: If a stage has no name or part, or names a part that has no goal.
    """
    stages = []
    for i, entry in enumerate(entries):
        name, part = entry.get('name'), entry.get('part')
        if not name or part not in parts or not parts[part].dynamic or part not in goal:
            raise TaskError(f'{where}[{i}]: a stage names itself and a dynamic part with a goal')
        stages.append((name, part))
    if len({name for name, _ in stages}) != len(stages):
        raise TaskError(f'{where}: two stages share a name')
    return tuple(stages)


def state_before(spec: Task, stage: str) -> dict[str, np.ndarray]:
    """Return the dynamic parts' world poses just before a stage: earlier stages' parts at goal.

    Raises:
        TaskError: If the task has no such stage.
    """
    names = [name for name, _ in spec.stages]
    if stage not in names:
        raise TaskError(f'{spec.name} has no stage {stage!r} (stages: {names})')
    done = {part for _, part in spec.stages[: names.index(stage)]}
    return {
        name: spec.states['goal' if name in done else 'start'][name]
        for name in spec.states['start']
    }


def placement(spec: Task, rng: np.random.Generator, xy_m: float, yaw_deg: float) -> np.ndarray:
    """Return a random move of a task's whole scene, a 4 x 4 world transform.

    Up to xy_m along x and along y, and up to yaw_deg about the vertical through the static parts'
    centre (the fixture's, the parts' starts' when there is none), each uniform.
    """
    poses = list(spec.fixed.values()) or list(spec.states['start'].values())
    centre = np.mean([T[:3, 3] for T in poses], axis=0)
    yaw = np.radians(rng.uniform(-yaw_deg, yaw_deg))
    c, s = np.cos(yaw), np.sin(yaw)
    move = np.eye(4)
    move[:2, :2] = [[c, -s], [s, c]]
    shift = np.array([*rng.uniform(-xy_m, xy_m, size=2), 0.0])
    move[:3, 3] = centre + shift - move[:3, :3] @ centre
    return move


def moved(spec: Task, move: np.ndarray) -> Task:
    """Return the task with its whole scene moved by a world transform.

    Every world pose moves with it: the static parts', each state's and the cable's segments.
    """
    cable = spec.cable
    if cable is not None:
        cable = dataclasses.replace(
            cable,
            centres=cable.centres @ move[:3, :3].T + move[:3, 3],
            rotations=move[:3, :3] @ cable.rotations,
        )
    return dataclasses.replace(
        spec,
        fixed={name: move @ T for name, T in spec.fixed.items()},
        states={
            state: {n: move @ T for n, T in poses.items()} for state, poses in spec.states.items()
        },
        cable=cable,
    )

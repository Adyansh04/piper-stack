"""A task's parts and cable as USD files, and their placement in a stage.

Each part becomes one USD file: its meshes in metres, their collision, its mass and its materials.
The cable becomes a chain of capsules jointed by springs that ends in its plug, laid along its
start path and straight at rest. The files are written once per content (a hash of task.yaml, the
meshes and the code that reads and writes them) under a cache directory and referenced from the
stage, so the simulator and Isaac Lab load the same assets. Runs where pxr and the PhysX schema are
(Isaac Sim, Isaac Lab).
"""

from __future__ import annotations

import hashlib
import logging
import shutil
from pathlib import Path

import numpy as np
import task as task_spec
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
from robot import prune_cache

log = logging.getLogger('sim.task_usd')

#: Finished conversions kept per task in the cache.
KEEP_CONVERSIONS = 3
#: PhysX's mesh collision approximation for each of task.yaml's collisions.
APPROXIMATIONS = {
    'sdf': 'sdf',
    'convex_hull': 'convexHull',
    'convex_decomposition': 'convexDecomposition',
}
#: USD's angular drive gains are per degree.
PER_DEGREE = np.pi / 180.0
#: Position iterations of the cable's bodies: a long jointed chain needs more than PhysX's 16.
CABLE_SOLVER_ITERATIONS = 64
#: A quarter turn about y: takes a joint's x axis (the cable's) onto the plug's -z (its back).
PLUG_JOINT_ROTATION = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])


def _matrix(T: np.ndarray) -> Gf.Matrix4d:
    """Return a 4 x 4 transform as USD's matrix, which acts on row vectors."""
    return Gf.Matrix4d(*np.asarray(T, dtype=float).T.flatten())


def _quat(R: np.ndarray) -> Gf.Quatf:
    """Return a rotation matrix as a USD quaternion."""
    T = np.eye(4)
    T[:3, :3] = R
    return Gf.Quatf(_matrix(T).ExtractRotationQuat())


def _set_pose(prim: Usd.Prim, T: np.ndarray) -> None:
    """Give a prim one transform op: the 4 x 4 transform T in its parent."""
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(_matrix(T))


def _new_stage(path: Path, root: str) -> tuple[Usd.Stage, Usd.Prim]:
    """Create a USD file (metres, z up) whose default prim is an Xform named root."""
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    prim = UsdGeom.Xform.Define(stage, Sdf.Path(f'/{root}')).GetPrim()
    stage.SetDefaultPrim(prim)
    return stage, prim


def _look(
    stage: Usd.Stage, path: Sdf.Path, colour: tuple[float, ...], metallic: float, roughness: float
) -> UsdShade.Material:
    """Define a preview-surface material of a colour, a metalness and a roughness."""
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path.AppendChild('surface'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return material


def _apply_newton(prim: Usd.Prim, schema: str, values: dict[str, tuple]) -> None:
    """Apply a Newton schema by name with its attributes (``name: (type, value)``).

    Newton's schemas are applied by name: the writer needs only pxr, and PhysX ignores them.
    """
    prim.AddAppliedSchema(schema)
    for name, (type_name, value) in values.items():
        prim.CreateAttribute(f'newton:{name}', type_name, custom=False).Set(value)


def _bind_physics_material(stage: Usd.Stage, prim: Usd.Prim, material: task_spec.Material) -> None:
    """Give a prim's colliders a material: friction, restitution and Newton's contact model."""
    physics = UsdShade.Material.Define(stage, prim.GetPath().AppendPath('Looks/physics'))
    api = UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
    api.CreateStaticFrictionAttr(material.static_friction)
    api.CreateDynamicFrictionAttr(material.dynamic_friction)
    api.CreateRestitutionAttr(material.restitution)
    real = Sdf.ValueTypeNames.Float
    _apply_newton(
        physics.GetPrim(),
        'NewtonMaterialAPI',
        {
            'contactStiffness': (real, material.contact_stiffness),
            'contactDamping': (real, material.contact_damping),
            'torsionalFriction': (real, material.torsional_friction_m),
            'rollingFriction': (real, material.rolling_friction_m),
        },
    )
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        physics, UsdShade.Tokens.weakerThanDescendants, 'physics'
    )


def _write_shape(
    stage: Usd.Stage, parent: Sdf.Path, shape: task_spec.Shape, contact: task_spec.Contact
) -> None:
    """Define a shape's mesh under a prim, with flat normals, its collision and its look.

    The collision is authored for PhysX and Newton alike: an SDF shape gets both engines' signed
    distance field settings.
    """
    name = shape.mesh.stem
    triangles = shape.triangles
    points, index = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    mesh = UsdGeom.Mesh.Define(stage, parent.AppendChild(name))
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(triangles), 3, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(index.astype(np.int32).ravel()))
    mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.repeat(normals, 3, 0).astype(np.float32)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    extent = np.stack((points.min(0), points.max(0))).astype(np.float32)
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(extent))
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*shape.colour)])
    prim = mesh.GetPrim()
    if shape.collision != 'none':
        UsdPhysics.CollisionAPI.Apply(prim)
        approximation = APPROXIMATIONS[shape.collision]
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
        real = Sdf.ValueTypeNames.Float
        _apply_newton(prim, 'NewtonCollisionAPI', {'contactGap': (real, contact.gap_m)})
        if shape.collision == 'sdf':
            sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf.CreateSdfResolutionAttr(shape.sdf_resolution)
            _apply_newton(
                prim,
                'NewtonSDFCollisionAPI',
                {
                    'sdfMaxResolution': (Sdf.ValueTypeNames.Int, shape.sdf_resolution),
                    'sdfNarrowBandInner': (real, -contact.sdf_band_m),
                    'sdfNarrowBandOuter': (real, contact.sdf_band_m),
                },
            )
    look = _look(
        stage, parent.AppendPath(f'Looks/{name}'), shape.colour, shape.metallic, shape.roughness
    )
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)


def _make_body(
    prim: Usd.Prim,
    mass_kg: float,
    *,
    com_m: np.ndarray,
    inertia_kg_m2: np.ndarray,
    principal_axes: np.ndarray,
) -> None:
    """Make a prim a rigid body with its full mass properties (Newton needs the inertia)."""
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr(mass_kg)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*com_m))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia_kg_m2))
    mass.CreatePrincipalAxesAttr(_quat(principal_axes))


def _make_part_body(prim: Usd.Prim, part: task_spec.Part) -> None:
    """Make a prim the rigid body of a dynamic part."""
    _make_body(
        prim,
        part.mass_kg,
        com_m=part.com_m,
        inertia_kg_m2=part.inertia_kg_m2,
        principal_axes=part.principal_axes,
    )


def write_part(part: task_spec.Part, task: task_spec.Task, path: Path) -> None:
    """Write a part as a USD file: its shapes in its own frame, a rigid body if dynamic.

    Args:
        part: The part.
        task: Its task, for the part's material and the contact settings.
        path: The file to write; it must not exist.
    """
    stage, prim = _new_stage(path, part.name)
    for shape in part.shapes:
        _write_shape(stage, prim.GetPath(), shape, task.contact)
    _bind_physics_material(stage, prim, task.materials[part.material])
    if part.dynamic:
        _make_part_body(prim, part)
    stage.Save()


def _define_joint(
    stage: Usd.Stage,
    path: Sdf.Path,
    bodies: tuple[Sdf.Path, Sdf.Path],
    frames: tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]],
    cable: task_spec.Cable,
    length_m: float,
) -> None:
    """Define a cable joint: a ball joint whose springs are the cable's over length_m.

    Twist is about the joint's x axis (PhysX's twist axis), bending about y and z. The jointed
    bodies do not collide (UsdPhysics' default), so overlapping neighbours stay quiet.
    """
    joint = UsdPhysics.Joint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([bodies[0]])
    joint.CreateBody1Rel().SetTargets([bodies[1]])
    (p0, R0), (p1, R1) = frames
    joint.CreateLocalPos0Attr(Gf.Vec3f(*p0))
    joint.CreateLocalRot0Attr(_quat(R0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(*p1))
    joint.CreateLocalRot1Attr(_quat(R1))
    prim = joint.GetPrim()
    for axis in ('transX', 'transY', 'transZ'):
        # A lower limit above the upper one locks the axis.
        limit = UsdPhysics.LimitAPI.Apply(prim, axis)
        limit.CreateLowAttr(1.0)
        limit.CreateHighAttr(-1.0)
    twist = cable.twist_stiffness_n_m2 / length_m
    bend = cable.bend_stiffness_n_m2 / length_m
    for axis, stiffness in (('rotX', twist), ('rotY', bend), ('rotZ', bend)):
        drive = UsdPhysics.DriveAPI.Apply(prim, axis)
        drive.CreateTypeAttr('force')
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateStiffnessAttr(stiffness * PER_DEGREE)
        drive.CreateDampingAttr(cable.damping_n_m_s * PER_DEGREE)


def segment_name(index: int) -> str:
    """Return the prim name of a cable segment, counted from the plug."""
    return f'segment_{index:03d}'


def write_cable(task: task_spec.Task, path: Path) -> None:
    """Write a task's cable and its plug as one USD file, laid along its start path.

    The segments (capsules along x) and the plug are rigid bodies in the world frame, jointed in
    maximal coordinates: PhysX keeps the authored bent shape as the start, where an articulation
    would rebuild it from joint positions. add() fixes the segment at the anchor. A segment's
    inertia is a rod's bending inertia about every axis: its true twist inertia is too small for
    the solver at the simulator's rate.

    Args:
        task: The task; it has a cable.
        path: The file to write; it must not exist.
    """
    cable = task.cable
    plug = task.parts[cable.plug]
    stage, root = _new_stage(path, 'cable')
    _bind_physics_material(stage, root, task.materials[plug.material])
    jacket = _look(stage, root.GetPath().AppendPath('Looks/jacket'), cable.colour, 0.0, 0.6)
    links = []
    for i, (centre, R, length) in enumerate(
        zip(cable.centres, cable.rotations, cable.lengths_m, strict=True)
    ):
        link = UsdGeom.Xform.Define(stage, root.GetPath().AppendChild(segment_name(i))).GetPrim()
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, centre
        _set_pose(link, T)
        mass_kg = cable.mass_kg_per_m * float(length)
        inertia = mass_kg * float(length) ** 2 / 12.0
        _make_body(
            link,
            mass_kg,
            com_m=np.zeros(3),
            inertia_kg_m2=np.full(3, inertia),
            principal_axes=np.eye(3),
        )
        iterations = PhysxSchema.PhysxRigidBodyAPI.Apply(link)
        iterations.CreateSolverPositionIterationCountAttr(CABLE_SOLVER_ITERATIONS)
        capsule = UsdGeom.Capsule.Define(stage, link.GetPath().AppendChild('capsule'))
        capsule.CreateAxisAttr(UsdGeom.Tokens.x)
        capsule.CreateRadiusAttr(cable.radius_m)
        capsule.CreateHeightAttr(float(length))
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(capsule.GetPrim()).Bind(jacket)
        links.append(link.GetPath())
    plug_prim = UsdGeom.Xform.Define(stage, root.GetPath().AppendChild(plug.name)).GetPrim()
    _set_pose(plug_prim, task.states['start'][plug.name])
    for shape in plug.shapes:
        _write_shape(stage, plug_prim.GetPath(), shape, task.contact)
    _make_part_body(plug_prim, plug)
    joints = root.GetPath().AppendChild('joints')
    UsdGeom.Scope.Define(stage, joints)
    lengths = cable.lengths_m
    identity = np.eye(3)
    for i in range(len(links) - 1):
        # body0 is the segment nearer the anchor; the joint sits where the two meet.
        _define_joint(
            stage,
            joints.AppendChild(f'joint_{i:03d}'),
            (links[i + 1], links[i]),
            (
                (np.array([-lengths[i + 1] / 2, 0, 0]), identity),
                (np.array([lengths[i] / 2, 0, 0]), identity),
            ),
            cable,
            float(lengths[i] + lengths[i + 1]) / 2.0,
        )
    _define_joint(
        stage,
        joints.AppendChild('joint_plug'),
        (links[0], plug_prim.GetPath()),
        ((np.array([-lengths[0] / 2, 0, 0]), identity), (cable.attach, PLUG_JOINT_ROTATION)),
        cable,
        float(lengths[0]),
    )
    stage.Save()


def build(task: task_spec.Task, cache: Path) -> dict[str, Path]:
    """Write a task's assets unless the cache holds them; return each one's USD file.

    The cable's plug is a body in the cable's file (``cable``), not a file of its own.

    Args:
        task: The task.
        cache: Directory of the conversions; each task keeps its newest few.

    Returns:
        The USD file of every part (and of the cable), by name.
    """
    digest = hashlib.sha1((task.root / 'task.yaml').read_bytes())
    for source in (Path(__file__), Path(task_spec.__file__)):
        digest.update(source.read_bytes())
    for shape in (s for p in task.parts.values() for s in p.shapes):
        digest.update(shape.mesh.read_bytes())
    key = digest.hexdigest()[:12]
    folder = cache / task.name / key
    plug = task.cable.plug if task.cable is not None else None
    files = {name: folder / f'{name}.usd' for name in task.parts if name != plug}
    if task.cable is not None:
        files['cable'] = folder / 'cable.usd'
    if (folder / 'done').is_file():
        return files
    # A conversion cut off before its marker is redone from scratch.
    shutil.rmtree(folder, ignore_errors=True)
    folder.mkdir(parents=True)
    log.info('Writing the %s task assets to %s', task.name, folder)
    for name, path in files.items():
        if name == 'cable':
            write_cable(task, path)
        else:
            write_part(task.parts[name], task, path)
    (folder / 'done').write_text(key)
    prune_cache(cache / task.name, KEEP_CONVERSIONS)
    return files


def add(
    stage: Usd.Stage,
    task: task_spec.Task,
    files: dict[str, Path],
    *,
    state: str = 'start',
    root: str = '/World/Task',
) -> dict[str, str]:
    """Place a task's assets in a stage at their poses in a state, the cable anchored.

    Args:
        stage: The stage.
        task: The task.
        files: The assets' USD files, from build().
        state: start or goal.
        root: The prim the parts go under.

    Returns:
        Each placed asset's prim path, by name.

    Raises:
        ValueError: If the state is unknown, or a task with a cable is placed in another state
            than start (the cable's path describes the start).
    """
    if state not in task.states:
        raise ValueError(f'State {state!r} is not one of {tuple(task.states)}')
    if task.cable is not None and state != 'start':
        raise ValueError(f'The {task.name} cable is laid for the start state only')
    poses = {**task.fixed, **task.states[state]}
    prims = {}
    for name, path in files.items():
        prim = stage.DefinePrim(Sdf.Path(f'{root}/{name}'), 'Xform')
        prim.GetReferences().AddReference(str(path))
        # The cable's file is laid out in the world frame.
        _set_pose(prim, np.eye(4) if name == 'cable' else poses[name])
        prims[name] = str(prim.GetPath())
    cable = task.cable
    if cable is not None:
        last = len(cable.lengths_m) - 1
        R = cable.rotations[last]
        end = cable.centres[last] + R[:, 0] * cable.lengths_m[last] / 2.0
        anchor = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(f'{prims["cable"]}/anchor_joint'))
        anchor.CreateBody1Rel().SetTargets([Sdf.Path(f'{prims["cable"]}/{segment_name(last)}')])
        anchor.CreateLocalPos0Attr(Gf.Vec3f(*end))
        anchor.CreateLocalRot0Attr(_quat(R))
        anchor.CreateLocalPos1Attr(Gf.Vec3f(float(cable.lengths_m[last]) / 2.0, 0.0, 0.0))
        anchor.CreateLocalRot1Attr(Gf.Quatf(1.0))
    return prims


def body_paths(task: task_spec.Task, prims: dict[str, str]) -> dict[str, str]:
    """Return the prim path of every dynamic part's rigid body, from add()'s prim paths."""
    paths = {name: prims[name] for name in task.states['start'] if name in prims}
    if task.cable is not None:
        paths[task.cable.plug] = f'{prims["cable"]}/{task.cable.plug}'
    return paths

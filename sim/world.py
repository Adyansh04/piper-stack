"""The table and the cubes on it, and the cubes' ground truth.

Lengths in metres. The arm's base stands at the world origin on the table's top (z 0), so the
robot's ``world`` frame is the simulator's.
"""

from __future__ import annotations

import logging
import threading

import numpy as np
from geometry_msgs.msg import TransformStamped
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

log = logging.getLogger('sim.world')

# Ground truth: every message counts for a test that compares poses, so a few queue.
QOS_GROUND_TRUTH = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def _pose_xyzw(T: np.ndarray) -> list[float]:
    """Return a 4 x 4 transform as its translation and quaternion: x, y, z, qx, qy, qz, qw."""
    q = Gf.Matrix4d(*np.asarray(T, dtype=float).T.flatten()).ExtractRotationQuat()
    return [*(float(v) for v in T[:3, 3]), *q.GetImaginary(), q.GetReal()]


def host_array(values) -> np.ndarray:
    """Return a physics tensor as a NumPy array: PhysX's CPU views give NumPy, Newton's Warp."""
    return values.numpy() if hasattr(values, 'numpy') else np.asarray(values)


def _box(
    stage: Usd.Stage, path: str, size_m: list[float], centre_m: list[float], colour: list[float]
) -> Usd.Prim:
    """Add a box collider of a size, at a centre, in a colour (RGB, 0 to 1)."""
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(cube)
    xform.SetTranslate(Gf.Vec3d(*centre_m))
    xform.SetScale(Gf.Vec3f(*size_m))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def _friction_material(stage: Usd.Stage, path: str, friction: dict) -> UsdShade.Material:
    """Return a physics material with the given static and dynamic friction."""
    material = UsdShade.Material.Define(stage, Sdf.Path(path))
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(float(friction['static']))
    api.CreateDynamicFrictionAttr(float(friction['dynamic']))
    api.CreateRestitutionAttr(0.0)
    return material


def build(stage: Usd.Stage, cfg: dict) -> list[tuple[str, str]]:
    """Add the light, the floor, the table and the cubes.

    Args:
        stage: The stage.
        cfg: sim.yaml's ``world`` section.

    Returns:
        Each cube's name and prim path.
    """
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    light = UsdLux.DomeLight.Define(stage, Sdf.Path('/World/DomeLight'))
    light.CreateIntensityAttr(float(cfg['light_intensity']))
    table = cfg['table']
    height_m = float(table['height_m'])
    length_m, width_m = (float(v) for v in table['size_m'])
    cx_m, cy_m = (float(v) for v in table['centre_m'])
    _box(stage, '/World/Floor', [6.0, 6.0, 0.02], [0.0, 0.0, -height_m - 0.01], cfg['floor_colour'])
    _box(
        stage,
        '/World/Table',
        [length_m, width_m, height_m],
        [cx_m, cy_m, -height_m / 2.0],
        table['colour'],
    )
    material = _friction_material(stage, '/World/Materials/cube', cfg['cube_friction'])
    cubes = []
    for cube in cfg['cubes']:
        size_m = float(cube['size_m'])
        x_m, y_m = (float(v) for v in cube['xy_m'])
        path = f'/World/Cubes/{cube["name"]}'
        prim = _box(stage, path, [size_m] * 3, [x_m, y_m, size_m / 2.0], cube['colour'])
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(cube['mass_kg']))
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, 'physics'
        )
        cubes.append((cube['name'], path))
    log.info('World: table %.2f m high, %d cubes', height_m, len(cubes))
    return cubes


class GroundTruth:
    """Publishes the objects' poses (cubes, a task's parts) in ``world`` on ``sim/objects``.

    Args:
        node: The simulator's node.
        objects: Each object's name and the prim path of its rigid body.
        period_s: Simulation time between two messages.
        fixed: Static objects' world poses (4 x 4 transforms) by name, sent with every message.
    """

    def __init__(
        self,
        node: Node,
        objects: list[tuple[str, str]],
        period_s: float,
        fixed: dict[str, np.ndarray] | None = None,
    ) -> None:
        fixed = fixed or {}
        self.names = [name for name, _ in objects] + list(fixed)
        self.paths = [path for _, path in objects]
        # As the physics views give them: x, y, z, qx, qy, qz, qw.
        self.fixed = np.array([_pose_xyzw(T) for T in fixed.values()]).reshape(-1, 7)
        self.period_s = period_s
        self.next_s = 0.0
        self.last_s = 0.0
        self.pub = node.create_publisher(TFMessage, 'sim/objects', QOS_GROUND_TRUTH)
        self.view = None
        self.sim_view = None

    def publish(self, t_s: float, sim_view) -> None:
        """Publish the poses if a period has passed; a new physics view rebuilds the body view."""
        # Stop and Play restart simulation time at 0.
        if t_s < self.last_s:
            self.next_s = 0.0
        self.last_s = t_s
        if not self.paths or sim_view is None or t_s < self.next_s:
            return
        if self.sim_view is not sim_view:
            self.view, self.sim_view = sim_view.create_rigid_body_view(self.paths), sim_view
        self.next_s = t_s + self.period_s
        poses = np.vstack((host_array(self.view.get_transforms()), self.fixed)).astype(float)
        stamp = Time(seconds=t_s).to_msg()
        msg = TFMessage()
        for name, pose in zip(self.names, poses, strict=True):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'world'
            tf.child_frame_id = name
            t, r = tf.transform.translation, tf.transform.rotation
            t.x, t.y, t.z = (float(v) for v in pose[:3])
            r.x, r.y, r.z, r.w = (float(v) for v in pose[3:7])
            msg.transforms.append(tf)
        self.pub.publish(msg)


def _like_view(values: np.ndarray, like):
    """Return values as a physics view takes them: as they are (NumPy) or as Warp on its device."""
    if not hasattr(like, 'device'):
        return values
    import warp as wp

    dtype = wp.float32 if values.dtype.kind == 'f' else wp.int32
    return wp.array(values, dtype=dtype, device=like.device)


class TaskReset:
    """Puts a task's moving parts back where a state has them, on ``sim/reset_task``.

    The state is the one before the stage named by the node's ``reset_stage`` parameter: the parts
    of earlier stages at their goals, the others at their starts; empty, the start. The service
    (std_srvs/Trigger) answers once the simulation loop has moved the parts, at rest.

    Args:
        node: The simulator's node.
        spec: The task (sim/task.py), as the simulator placed it.
        bodies: Each moving part's name and the prim path of its rigid body.
        fixed_starts: Other bodies a reset puts back where they start, whatever the state: each
            prim path and its world pose (a cable's segments).
        timeout_s: How long the service waits for the loop, in wall seconds.
    """

    def __init__(
        self,
        node: Node,
        spec,
        bodies: dict[str, str],
        fixed_starts: dict[str, np.ndarray] | None = None,
        timeout_s: float = 5.0,
    ) -> None:
        self.spec = spec
        self.names = list(bodies)
        self.paths = [bodies[name] for name in self.names] + list(fixed_starts or {})
        # As the physics views take them: x, y, z, qx, qy, qz, qw.
        self.fixed_starts = np.array(
            [_pose_xyzw(T) for T in (fixed_starts or {}).values()]
        ).reshape(-1, 7)
        self.timeout_s = timeout_s
        #: Guards ``pending``, set by the service (rclpy thread) and taken by the loop.
        self.lock = threading.Lock()
        self.pending: np.ndarray | None = None
        self.applied = threading.Event()
        self.view = None
        self.sim_view = None
        self.node = node
        node.declare_parameter('reset_stage', '')
        node.create_service(Trigger, 'sim/reset_task', self.on_reset)

    def on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Hand the loop the state's poses and wait for it to set them (rclpy thread)."""
        import task as task_spec

        stage = self.node.get_parameter('reset_stage').value
        try:
            poses = task_spec.state_before(self.spec, stage) if stage else self.spec.states['start']
        except task_spec.TaskError as error:
            response.success, response.message = False, str(error)
            return response
        self.applied.clear()
        with self.lock:
            self.pending = np.vstack(
                (np.array([_pose_xyzw(poses[name]) for name in self.names]), self.fixed_starts)
            )
        response.success = self.applied.wait(self.timeout_s)
        response.message = (
            f'Parts set {"before " + stage if stage else "at the start"}'
            if response.success
            else 'The simulation loop did not take the reset: is the simulation running?'
        )
        return response

    def apply(self, sim_view) -> None:
        """Set the parts where a pending reset puts them, at rest (simulation loop)."""
        if sim_view is None:
            return
        with self.lock:
            poses, self.pending = self.pending, None
        if poses is None:
            return
        if self.sim_view is not sim_view:
            self.view, self.sim_view = sim_view.create_rigid_body_view(self.paths), sim_view
        like = self.view.get_transforms()
        indices = _like_view(np.arange(len(self.paths), dtype=np.int32), like)
        self.view.set_transforms(_like_view(poses.astype(np.float32), like), indices)
        self.view.set_velocities(
            _like_view(np.zeros((len(self.paths), 6), dtype=np.float32), like), indices
        )
        self.applied.set()

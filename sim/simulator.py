"""The simulated arm's joint loop, the world and the run loop.

Imported by sim.py once Isaac Sim runs and its ROS 2 bridge is enabled (rclpy comes with the
bridge). The joint loop runs in Isaac Sim's physics callback at ``app.physics_hz``: it takes the
stack's position targets from ``isaac_joint_commands`` and answers on ``isaac_joint_states``, the
topics of ros-controls' joint_state_topic_hardware_interface (``hardware:=isaac``). In shadow mode
the arm is a digital twin instead: it takes the joint positions another stack measures and
publishes its own beside them. Its cost per step is in the status line.
"""

from __future__ import annotations

import math
import threading
import time
import traceback
from collections import deque
from pathlib import Path

import carb
import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.usd
import rclpy
import robot
import sensors
import task as task_spec
import task_usd
import world
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from omni.kit.loop import _loop
from pxr import Sdf, UsdPhysics
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

# /clock: only the newest stamp matters; reliable also serves tools on the default QoS.
QOS_CLOCK = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
# Joint states and commands: only the newest counts. A reliable publisher also serves best-effort
# subscribers.
QOS_JOINTS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
# The robot USD's physics variant for each engine (the importer writes physx and mujoco, each on
# top of plain UsdPhysics).
PHYSICS_VARIANTS = {'physx': 'physx', 'newton': 'mujoco'}
# Physics steps the published velocity spans (25 ms at 200 Hz): a single step's difference carries
# the contact solver's jitter, which a finger pressing on an object turns into motion that never
# stops (the gripper controller's stall check).
VELOCITY_WINDOW_STEPS = 5


class StepStats:
    """Wall time the physics callback takes, averaged between two status lines."""

    def __init__(self) -> None:
        self.total_s = 0.0
        self.steps = 0

    def add(self, duration_s: float) -> None:
        """Count one step that took duration_s."""
        self.total_s += duration_s
        self.steps += 1

    def take_mean_ms(self) -> float:
        """Return the mean step time in milliseconds since the last call, and start over."""
        mean_ms = 1000.0 * self.total_s / max(self.steps, 1)
        self.total_s, self.steps = 0.0, 0
        return mean_ms


class Arm:
    """The arm and its gripper, through the robot's articulation.

    The joints follow ``isaac_joint_commands`` (position targets); a joint the URDF makes mimic
    another follows it as the mimic says; every joint's state goes to ``isaac_joint_states``. As a
    twin (``shadow``) the joints are set to the positions on the shadow section's
    ``joint_states_topic`` every step, at rest, and their state goes to its ``state_topic``.

    Args:
        node: The simulator's node.
        articulation_root: Prim path of the robot's articulation root.
        urdf: The robot's URDF text, for its mimic tags.
        engine: The physics engine (physx or newton), whose tensor backend the views use.
        shadow: sim.yaml's ``shadow`` section in shadow mode; None when the stack drives the arm.
    """

    def __init__(
        self,
        node: Node,
        *,
        articulation_root: str,
        urdf: str,
        engine: str,
        shadow: dict | None = None,
    ) -> None:
        self.log = node.get_logger()
        self.articulation_root = articulation_root
        self.engine = engine
        self.urdf = urdf
        self.shadow = shadow is not None
        state_topic = shadow['state_topic'] if shadow else 'isaac_joint_states'
        command_topic = shadow['joint_states_topic'] if shadow else 'isaac_joint_commands'
        self.pub = node.create_publisher(JointState, state_topic, QOS_JOINTS)
        node.create_subscription(JointState, command_topic, self.on_command, QOS_JOINTS)
        #: Guards ``command``, written by the rclpy thread and read by the physics callback.
        self.lock = threading.Lock()
        self.command: dict[str, float] = {}
        self.view = None
        self.sim_view = None
        self.last_error = ''
        #: The last VELOCITY_WINDOW_STEPS + 1 positions and their times, for the velocity.
        self.history: deque[tuple[float, np.ndarray]] = deque(maxlen=VELOCITY_WINDOW_STEPS + 1)

    def on_command(self, msg: JointState) -> None:
        """Take position targets (rclpy thread); non-finite ones are ignored."""
        finite = {n: p for n, p in zip(msg.name, msg.position, strict=False) if math.isfinite(p)}
        if len(finite) < len(msg.name):
            self.log.warning('Joint command with missing or non-finite positions: kept the old')
        with self.lock:
            self.command.update(finite)

    def valid_sim_view(self):
        """Return the physics view the articulation is on, or None while there is none."""
        return self.sim_view if self.view is not None and self.sim_view.is_valid else None

    def ensure(self) -> None:
        """Build the physics views of the articulation when there are none or they went stale.

        Raises:
            Exception: Whatever the physics tensors raise while physics is stopped.
        """
        if self.view is not None and self.sim_view.is_valid:
            return
        import omni.physics.tensors as tensors

        # PhysX runs the arm on the CPU (NumPy views), Newton on the GPU (Warp views).
        self.sim_view = tensors.create_simulation_view(
            'numpy' if self.engine == 'physx' else 'warp',
            stage_id=omni.usd.get_context().get_stage_id(),
            backend=self.engine,
        )
        self.sim_view.set_subspace_roots('/')
        view = self.sim_view.create_articulation_view(self.articulation_root)
        self.names = list(view.shared_metatype.dof_names)
        self.index = {n: i for i, n in enumerate(self.names)}
        self.mimics = []
        for name in self.names:
            mimic = robot.mimic_of(self.urdf, name)
            if mimic is not None and mimic[0] in self.index:
                self.mimics.append((self.index[name], self.index[mimic[0]], mimic[1], mimic[2]))
        self.free = [i for i in range(len(self.names)) if i not in {m[0] for m in self.mimics}]
        self.device = None
        if self.engine != 'physx':
            self.device = view.get_dof_positions().device
        self.articulations = self.to_view(np.array([0], dtype=np.int32))
        self.history.clear()
        self.view = view
        self.log.info(f'Arm articulation view ready: joints {self.names}')

    def step(self, t_s: float) -> None:
        """Run one control step after a physics step; a failure drops the views for a rebuild."""
        if self.view is None:
            return
        try:
            self._control(t_s)
            self.last_error = ''
        except Exception as error:  # noqa: BLE001 - the physics callback must survive a stale view
            if str(error) != self.last_error:
                self.last_error = str(error)
                self.log.warning(f'Arm step failed, rebuilding the views: {traceback.format_exc()}')
            self.view = None

    def to_view(self, values: np.ndarray):
        """Return a NumPy array as the views take it: itself, or a Warp array on their device."""
        if self.device is None:
            return values
        import warp as wp

        dtype = wp.float32 if values.dtype.kind == 'f' else wp.int32
        return wp.array(values, dtype=dtype, device=self.device)

    def _control(self, t_s: float) -> None:
        with self.lock:
            command = dict(self.command)
        positions = world.host_array(self.view.get_dof_positions())
        targets = world.host_array(self.view.get_dof_position_targets()).copy()
        for name, value in command.items():
            if name in self.index:
                targets[0, self.index[name]] = value
        for follower, leader, multiplier, offset in self.mimics:
            targets[0, follower] = multiplier * targets[0, leader] + offset
        if self.shadow and command:
            # The twin shows where the other arm is: set there at rest, the drives holding it.
            self.view.set_dof_positions(self.to_view(targets), self.articulations)
            self.view.set_dof_velocities(self.to_view(np.zeros_like(targets)), self.articulations)
            positions = targets
        self.view.set_dof_position_targets(self.to_view(targets), self.articulations)
        q = positions[0].astype(float)
        # PhysX's velocities stay off zero on a joint its drive holds against a load: the change
        # in position over the window is the motion.
        self.history.append((t_s, q))
        t0, q0 = self.history[0]
        qd = np.zeros_like(q) if t_s <= t0 else (q - q0) / (t_s - t0)
        effort = world.host_array(self.view.get_dof_projected_joint_forces())[0].astype(float)
        msg = JointState(name=[self.names[i] for i in self.free])
        msg.header.stamp = Time(seconds=t_s).to_msg()
        msg.position = q[self.free].tolist()
        msg.velocity = qd[self.free].tolist()
        msg.effort = effort[self.free].tolist()
        self.pub.publish(msg)


def configure_newton(cfg: dict, physics_hz: float) -> None:
    """Set Newton up for the next play: MuJoCo-Warp with Newton's own (signed distance) contacts.

    Args:
        cfg: sim.yaml's ``newton`` section.
        physics_hz: Physics steps a second; each runs ``substeps`` solver steps.
    """
    from isaacsim.physics.newton import (
        MuJoCoSolverConfig,
        NewtonConfig,
        NewtonSolverOutputConfig,
    )
    from isaacsim.physics.newton import configure_newton as configure

    configure(
        NewtonConfig(
            num_substeps=int(cfg['substeps']),
            physics_frequency=float(physics_hz),
            solver_cfg=MuJoCoSolverConfig(
                cone=cfg['cone'], njmax=int(cfg['njmax']), nconmax=int(cfg['nconmax'])
            ),
            # The joint loop publishes every joint's effort: the contact tasks' torque estimate.
            solver_outputs=NewtonSolverOutputConfig(link_incoming_joint_force=True),
        )
    )


def find_body(stage, under: str, name: str) -> str:
    """Return the prim path of the rigid body called name below a prim.

    Raises:
        RuntimeError: If there is none.
    """
    for prim in stage.Traverse():
        if (
            str(prim.GetPath()).startswith(under)
            and prim.GetName() == name
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ):
            return str(prim.GetPath())
    raise RuntimeError(f'No rigid body {name} below {under}')


def find_articulation_root(stage, under: str) -> str:
    """Return the prim path of the first articulation root below a prim.

    Raises:
        RuntimeError: If there is none.
    """
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(under) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError(f'No articulation root below {under}')


def simulate(
    app,
    cfg: dict,
    arm: Arm,
    truth: world.GroundTruth,
    stats: StepStats,
    log,
    stop: threading.Event,
) -> None:
    """Run frames until the app closes or stop is set; log the status every status_period_s."""
    status_period_s = cfg['app']['status_period_s']
    wall0, sim0, updates = time.perf_counter(), SimulationManager.get_simulation_time(), 0
    ensure_error = ''
    deadline = cfg['app'].get('run_s')
    started = time.perf_counter()
    while app.is_running() and not stop.is_set():
        try:
            arm.ensure()
            ensure_error = ''
        except Exception as error:  # noqa: BLE001 - the views cannot be built while physics stops
            if repr(error) != ensure_error:
                ensure_error = repr(error)
                log.warning(f'Arm articulation view not ready, retrying: {ensure_error}')
        app.update()
        updates += 1
        truth.publish(SimulationManager.get_simulation_time(), arm.valid_sim_view())
        now = time.perf_counter()
        if now - wall0 > status_period_s:
            sim_now = SimulationManager.get_simulation_time()
            log.info(
                f'Simulation time {sim_now:.1f} s, '
                f'real-time factor {(sim_now - sim0) / (now - wall0):.2f}, '
                f'{updates / (now - wall0):.1f} frames/s, '
                f'physics callback {stats.take_mean_ms():.3f} ms per step'
            )
            wall0, sim0, updates = now, sim_now, 0
        if deadline is not None and now - started > deadline:
            log.info(f'Ran for {deadline} s: stopping')
            break


def run(app, cfg: dict, stop: threading.Event) -> None:
    """Build the world and the robot, then simulate until the app closes or stop is set.

    Args:
        app: The running SimulationApp, with the ROS 2 bridge enabled.
        cfg: The whole sim.yaml.
        stop: Set by a signal to end the run.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('piper_sim')
    log = node.get_logger()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name='rclpy', daemon=True)
    try:
        stage = omni.usd.get_context().get_stage()
        shadow = cfg['shadow'] if cfg['app']['mode'] == 'shadow' else None
        r = cfg['robot']
        task = cfg['world']['task']
        # A twin's cubes would stand where nothing is on the real table; a task replaces them.
        objects = world.build(
            stage, {**cfg['world'], 'cubes': []} if shadow or task else cfg['world']
        )
        if task and not shadow:
            spec = task_spec.load(Path(cfg['world']['tasks_dir']) / task)
            files = task_usd.build(spec, Path(r['usd_cache']).parent / 'tasks')
            bodies = task_usd.body_paths(spec, task_usd.add(stage, spec, files))
            objects += list(bodies.items())
            log.info(f'Task {task}: {len(spec.parts)} parts, {len(bodies)} moving')
        usd, urdf = robot.import_robot(
            Path(r['urdf']), Path(r['usd_cache']), r['packages'], r['drives']
        )
        prim = stage.DefinePrim(Sdf.Path('/World/piper'), 'Xform')
        prim.GetReferences().AddReference(usd)
        # The importer's USD leaves its physics variant unselected: no bodies, no joints.
        prim.GetVariantSet('Physics').SetVariantSelection(PHYSICS_VARIANTS[cfg['app']['engine']])
        if cfg['app']['engine'] == 'newton':
            count = robot.set_mujoco_gains(stage, '/World/piper', r['drives'])
            log.info(f'MuJoCo actuators given the drives of sim.yaml: {count}')
        app.update()
        root = find_articulation_root(stage, '/World/piper')
        arm = Arm(
            node, articulation_root=root, urdf=urdf, engine=cfg['app']['engine'], shadow=shadow
        )
        truth = world.GroundTruth(node, objects, float(cfg['world']['ground_truth_period_s']))
        camera = cfg['camera']
        if shadow:
            camera = {**camera, 'namespace': shadow['camera_namespace']}
        if camera['enabled']:
            body, T = robot.frame_in_body(urdf, camera['frame'])
            sensors.add_camera(
                stage,
                f'{find_body(stage, "/World/piper", body)}/wrist_camera',
                T,
                camera,
                float(cfg['app']['render_hz']),
            )
        overhead = cfg['overhead_camera']
        # A twin's overhead view is the real camera's.
        if overhead['enabled'] and not shadow:
            body, T = robot.frame_in_body(urdf, overhead['frame'])
            sensors.add_camera(
                stage,
                f'{find_body(stage, "/World/piper", body)}/overhead_camera',
                T,
                overhead,
                float(cfg['app']['render_hz']),
            )
        pub_clock = node.create_publisher(Clock, '/clock', QOS_CLOCK)
        stats = StepStats()

        def on_physics_step(_dt_s: float, _context) -> None:
            started = time.perf_counter()
            t_s = SimulationManager.get_simulation_time()
            # A twin runs beside a stack on the wall clock: its clock would confuse that stack.
            if shadow is None:
                pub_clock.publish(Clock(clock=Time(seconds=t_s).to_msg()))
            arm.step(t_s)
            stats.add(time.perf_counter() - started)

        physics_hz, render_hz = cfg['app']['physics_hz'], cfg['app']['render_hz']
        engine = cfg['app']['engine']
        # Newton registers itself but stays inactive without its auto-switch (off: sim.py).
        if engine == 'newton':
            configure_newton(cfg['newton'], physics_hz)
            SimulationManager.switch_physics_engine('newton')
        device = cfg['app']['device'][engine]
        SimulationManager.setup_simulation(dt=1.0 / physics_hz, device=device)
        loop_runner = _loop.acquire_loop_interface()
        loop_runner.set_manual_step_size(1.0 / render_hz)
        loop_runner.set_manual_mode(True)
        settings = carb.settings.get_settings()
        # Each frame advances 1 / render_hz of simulation time: pacing frames below render_hz runs
        # the simulation slower than wall time.
        frame_rate_hz = render_hz * float(cfg['app']['real_time_factor'])
        settings.set('/app/runLoops/main/rateLimitFrequency', frame_rate_hz)
        settings.set('/app/runLoops/main/rateLimitEnabled', bool(cfg['app']['real_time']))
        settings.set('/persistent/simulation/minFrameRate', 1)
        SimulationManager.register_callback(
            on_physics_step, event=SimulationEvent.PHYSICS_POST_STEP
        )
        spin_thread.start()
        app_utils.play()
        for _ in range(3):
            app.update()
        log.info(
            f'Running ({cfg["app"]["mode"]}): physics {physics_hz} Hz on {device} '
            f'({SimulationManager.get_active_physics_engine()}), render {render_hz} Hz, '
            f'real time {cfg["app"]["real_time"]}, articulation {root}'
        )
        simulate(app, cfg, arm, truth, stats, log, stop)
        log.info('Shutting down')
    finally:
        app_utils.stop()
        executor.shutdown(timeout_sec=5.0)
        if spin_thread.is_alive():
            spin_thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()

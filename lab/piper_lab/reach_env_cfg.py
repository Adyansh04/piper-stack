"""The Piper's reach task: the tool centre point to a commanded pose over the table.

After Isaac Lab's deployed UR10e reach task (``isaaclab_tasks.contrib.deploy.reach``), on the
simulator's own robot USD, joint drives and table (``sim/robot.py``, ``sim/config/sim.yaml``), so a
policy trained here drives the arm that Isaac Sim simulates for the stack. The joints, the
action's offsets and scale, the motion caps and the rates are those of the stack's policy runner
(``piper_controllers``' policy_runner.yaml), which runs the exported policy through MoveIt Servo:
50 Hz policy steps on a 200 Hz ramp. The policy observes its joints (positions in radians,
velocities in rad/s) and the target pose in the base frame (position, then the quaternion x, y, z,
w), and asks for those joints' positions, which :mod:`piper_lab.actions` approaches under the
caps.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import isaaclab.sim as sim_utils
import isaaclab_tasks.contrib.deploy.mdp as mdp
import torch
import yaml
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg, OffsetCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_from_matrix
from isaaclab.utils.noise import UniformNoiseCfg as Unoise
from isaaclab_tasks.contrib.deploy.reach.reach_env_cfg import ReachEnvCfg

from piper_lab.actions import RampedJointPositionActionCfg

#: The repository: ``sim/`` holds the simulator's robot import and configuration.
ROOT = Path(__file__).resolve().parents[2]
#: The stack's policy runner configuration: it sets the joints, the action's offsets and scale, the
#: motion caps and the rates the task trains with.
RUNNER_CONFIG = ROOT / 'ros_ws' / 'src' / 'piper_controllers' / 'config' / 'policy_runner.yaml'
#: The importer nests each moving link under its parent and merges fixed ones into theirs
#: (base_link into world, gripper_base and the tcp into link6).
BASE_PATH = '{ENV_REGEX_NS}/Robot/Geometry/world'
TOOL_BODY = 'link6'
TOOL_PATH = f'{BASE_PATH}/link1/link2/link3/link4/link5/{TOOL_BODY}'
#: Thickness of the table top, in metres; only its surface (z 0) matters.
TABLE_THICKNESS_M = 0.05
#: Tilt of the target from pointing straight down, in radians: the wrist (joint5) turns +-70 deg.
TILT_RAD = math.radians(15.0)
#: Distance of the reward's keypoints from the tool, in metres: the template's 0.45 m suits the
#: UR10e's 1.3 m reach; at a third of that, orientation errors no longer swamp position errors.
KEYPOINT_SCALE_M = 0.15


def _sim_robot():
    """Return the simulator's ``sim/robot.py``, loaded by path since sim/ is no package."""
    spec = importlib.util.spec_from_file_location('piper_sim_robot', ROOT / 'sim' / 'robot.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def policy_runner() -> dict:
    """Return the policy runner's parameters, as ``piper_controllers`` configures them."""
    return yaml.safe_load(RUNNER_CONFIG.read_text())['policy_runner']['ros__parameters']


def _arm(joints: list[str]) -> SceneEntityCfg:
    """Return the arm's joints as a scene entity; one each, since a manager resolves it in place."""
    return SceneEntityCfg('robot', joint_names=joints, preserve_order=True)


def piper_cfg(robot_section: dict, start: dict[str, float]) -> tuple[ArticulationCfg, OffsetCfg]:
    """Return the arm on the simulator's USD and drives, and the TCP's offset in the tool body.

    Args:
        robot_section: ``sim.yaml``'s ``robot`` section (URDF, USD cache, packages, drives).
        start: Joint positions an episode starts from, in rad, by joint name.

    Returns:
        The articulation, its gains left to the USD (the importer wrote sim.yaml's drives into it),
        and the ``tcp`` frame's transform in :data:`TOOL_BODY`.

    Raises:
        ValueError: If the importer merges the ``tcp`` frame into another body than
            :data:`TOOL_BODY`.
    """
    robot = _sim_robot()
    usd, urdf = robot.import_robot(
        Path(robot_section['urdf']),
        Path(robot_section['usd_cache']),
        robot_section['packages'],
        robot_section['drives'],
    )
    body, tcp = robot.frame_in_body(urdf, 'tcp')
    if body != TOOL_BODY:
        raise ValueError(f'The tcp frame is merged into {body}, the task expects {TOOL_BODY}')
    rotation = quat_from_matrix(torch.tensor(tcp[:3, :3], dtype=torch.float32))
    offset = OffsetCfg(pos=tuple(float(v) for v in tcp[:3, 3]), rot=tuple(rotation.tolist()))
    articulation = ArticulationCfg(
        prim_path='{ENV_REGEX_NS}/Robot',
        spawn=sim_utils.UsdFileCfg(usd_path=usd, variants={'Physics': 'physx'}),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos={**start, 'joint[78]': 0.0}),
        actuators={
            'joints': ImplicitActuatorCfg(
                joint_names_expr=['joint.*'], stiffness=None, damping=None
            )
        },
    )
    return articulation, offset


@configclass
class PiperReachEnvCfg(ReachEnvCfg):
    """The TCP to a pose 3 to 10 cm above the table in front of the arm, pointing down."""

    def __post_init__(self):
        """Fill the template with the Piper, its table, actions, targets and randomisation."""
        super().__post_init__()
        sim_cfg = yaml.safe_load((ROOT / 'sim' / 'config' / 'sim.yaml').read_text())
        controller = policy_runner()
        joints = controller['joints']
        start = dict(zip(joints, controller['action_offset'], strict=True))
        self.scene.robot, tcp = piper_cfg(sim_cfg['robot'], start)
        # The importer anchors the base by a fixed joint to the robot's own Xform, which PhysX's
        # replication of env 0 does not carry over (the copies float and blow up): every env is
        # parsed on its own instead, at some seconds more start-up.
        self.scene.replicate_physics = False

        # The template's ground and stand are Nucleus assets; the arm stands on the sim's table.
        self.scene.ground = None
        table = sim_cfg['world']['table']
        self.scene.table = AssetBaseCfg(
            prim_path='{ENV_REGEX_NS}/Table',
            spawn=sim_utils.CuboidCfg(
                size=(*table['size_m'], TABLE_THICKNESS_M),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(table['colour'])),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=(*table['centre_m'], -TABLE_THICKNESS_M / 2)
            ),
        )
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path=BASE_PATH,
            target_frames=[
                FrameTransformerCfg.FrameCfg(prim_path=TOOL_PATH, name='end_effector', offset=tcp)
            ],
        )

        self.actions.arm_action = RampedJointPositionActionCfg(
            asset_name='robot',
            joint_names=joints,
            preserve_order=True,
            scale=controller['action_scale'],
            max_velocity=controller['max_velocity'],
            max_acceleration=controller['max_acceleration'],
        )

        for term in (
            self.rewards.end_effector_keypoint_tracking,
            self.rewards.end_effector_keypoint_tracking_exp,
        ):
            term.params['keypoint_scale'] = KEYPOINT_SCALE_M

        # Targets in the base frame, inside the straight-down reach map (tests/reach-map-1.log).
        command = self.commands.ee_pose
        command.body_name = TOOL_BODY
        command.debug_vis = False
        # Two targets an episode: at 0.5 rad/s^2, joint1's swing across the box takes about 3.5 s.
        command.resampling_time_range = (6.0, 6.0)
        command.ranges.pos_x = (0.15, 0.30)
        command.ranges.pos_y = (-0.15, 0.15)
        command.ranges.pos_z = (0.03, 0.10)
        command.ranges.roll = (math.pi - TILT_RAD, math.pi + TILT_RAD)
        command.ranges.pitch = (-TILT_RAD, TILT_RAD)
        command.ranges.yaw = (-math.pi / 2, math.pi / 2)

        # Joint readings with the CAN feedback's noise and up to one policy step (20 ms) late.
        policy = self.observations.policy
        policy.joint_pos.params = {'asset_cfg': _arm(joints)}
        policy.joint_pos.noise = Unoise(n_min=-0.005, n_max=0.005)
        policy.joint_vel.params = {'asset_cfg': _arm(joints)}
        policy.joint_vel.noise = Unoise(n_min=-0.02, n_max=0.02)
        for term in (policy.joint_pos, policy.joint_vel):
            term.delay_min_lag = 0
            term.delay_max_lag = 1

        events = self.events
        for term in (
            events.reset_robot_joints,
            events.robot_joint_stiffness_and_damping,
            events.joint_friction,
        ):
            term.params['asset_cfg'] = _arm(joints)
        self.events.link_mass = EventTerm(
            func=mdp.randomize_rigid_body_mass,
            mode='startup',
            params={
                'asset_cfg': SceneEntityCfg('robot', body_names='link[1-6]'),
                'mass_distribution_params': (0.8, 1.2),
                'operation': 'scale',
            },
        )

        # The stack's joint loop: physics at its rate (200 Hz), a policy step every few.
        physics_hz = sim_cfg['app']['physics_hz']
        self.sim.dt = 1.0 / physics_hz
        # The runner ramps at jog_rate_hz as training does at the physics rate.
        if controller['jog_rate_hz'] != physics_hz:
            raise ValueError(f'jog_rate_hz must be the physics rate of {physics_hz} Hz')
        self.decimation = round(physics_hz / controller['policy_rate_hz'])
        if self.decimation < 1 or self.decimation * controller['policy_rate_hz'] != physics_hz:
            raise ValueError(f'policy_rate_hz must divide the physics rate of {physics_hz} Hz')
        self.sim.render_interval = self.decimation

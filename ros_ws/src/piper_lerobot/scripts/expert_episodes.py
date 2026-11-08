#!/usr/bin/env python3
"""Run a task's stage again and again in the simulator, marking each run as a LeRobot episode.

Each episode: the arm goes to one of the SRDF poses at random (the skills' move_arm_to_named_pose),
the simulator puts the task's parts as they are just before the stage (sim/reset_task, after its
reset_stage parameter), the marker says ``record:<n>:<task>``, the skills run the stage
(run_stage), and the marker says ``keep:<n>`` when the stage succeeded or ``discard:<n>``.
piper_lerobot's recorder (lerobot/record_episodes.py, in LeRobot's environment) records the
episodes the marker cuts. Needs the simulator with the task, the stack with MoveIt and the task
scene, the skills with the task and the bridge:

    ros2 run piper_lerobot expert_episodes.py --stage ring --episodes 50 \\
        --task "Put the ring on the post" --ros-args -p use_sim_time:=true

With --policy, a trained LeRobot policy runs each episode instead of the skills (lerobot-rollout
with the piper_ros robot, through the bridge and MoveIt Servo, which needs servo:=true): the
stage's contacts are allowed in MoveIt's scene for its duration, and an episode succeeds when the
part ends at its goal on its mate (the simulator's ground truth).

Python: it sequences ROS services and actions a few times a minute (data generation).
"""

from __future__ import annotations

import argparse
import math
import os
import random
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from piper_msgs.action import MoveArmToNamedPose, RunStage
from piper_msgs.srv import AllowContacts
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

#: The links a policy's gripper touches parts and their supports with, and what it comes near.
GRIPPER_LINKS = ('gripper_base', 'link6', 'link7', 'link8')
#: LeRobot's command line with the Piper's plugins, in LeRobot's own environment.
LEROBOT = ('/opt/lerobot-venv/bin/python', 'piper_lerobot.py')
LEROBOT_DIR = Path('/piper-stack/ros_ws/src/piper_lerobot/lerobot')

#: The marker is latched: a recorder that starts late still reads the current one.
QOS_MARKER = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class Expert:
    """The clients of one run of episodes.

    Args:
        node: The node that calls them.
        timeout_s: The longest wait for a service or a server, in s.
    """

    def __init__(self, node: Node, timeout_s: float) -> None:
        self.node = node
        self.timeout_s = timeout_s
        self.marker = node.create_publisher(String, '/lerobot/episode', QOS_MARKER)
        self.reset = node.create_client(Trigger, '/sim/reset_task')
        self.set_parameters = node.create_client(SetParameters, '/piper_sim/set_parameters')
        self.stage = ActionClient(node, RunStage, '/run_stage')
        self.pose = ActionClient(node, MoveArmToNamedPose, '/move_arm_to_named_pose')
        self.allow = node.create_client(AllowContacts, '/task_scene/allow_contacts')
        self.poses: dict[str, np.ndarray] = {}
        node.create_subscription(TFMessage, '/sim/objects', self.on_poses, 10)

    def on_poses(self, msg: TFMessage) -> None:
        """Keep the simulator's latest pose of each object (4 x 4, world)."""
        for tf in msg.transforms:
            t, q = tf.transform.translation, tf.transform.rotation
            T = np.eye(4)
            T[:3, :3] = quaternion_matrix(q.x, q.y, q.z, q.w)
            T[:3, 3] = (t.x, t.y, t.z)
            self.poses[tf.child_frame_id] = T

    def allow_contacts(self, pairs: list[tuple[str, str]], allowed: bool) -> None:
        """Allow or forbid contacts in MoveIt's scene.

        Raises:
            RuntimeError: If task_scene refuses.
        """
        request = AllowContacts.Request(
            first=[a for a, _ in pairs], second=[b for _, b in pairs], allowed=allowed
        )
        response = self.call(self.allow, request)
        if not response.success:
            raise RuntimeError(f'task_scene: {response.message}')

    def mark(self, text: str) -> None:
        """Give the episode marker."""
        self.marker.publish(String(data=text))

    def call(self, client, request):
        """Call a service and return its response.

        Raises:
            RuntimeError: If the service is not offered or does not answer in time.
        """
        if not client.wait_for_service(timeout_sec=self.timeout_s):
            raise RuntimeError(f'{client.srv_name} is not offered')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout_s)
        if future.result() is None:
            raise RuntimeError(f'{client.srv_name} did not answer')
        return future.result()

    def run(self, client: ActionClient, goal, limit_s: float):
        """Send an action goal and return its result, or None if refused or late.

        Raises:
            RuntimeError: If the action server is not up.
        """
        if not client.wait_for_server(timeout_sec=self.timeout_s):
            raise RuntimeError(f'{client._action_name} is not offered')
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=self.timeout_s)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return None
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, done, timeout_sec=limit_s)
        if done.result() is None:
            handle.cancel_goal_async()
            return None
        return done.result().result

    def reset_before(self, stage: str) -> None:
        """Put the parts as they are just before a stage.

        Raises:
            RuntimeError: If the simulator refuses or does not answer.
        """
        value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=stage)
        self.call(
            self.set_parameters,
            SetParameters.Request(parameters=[Parameter(name='reset_stage', value=value)]),
        )
        response = self.call(self.reset, Trigger.Request())
        if not response.success:
            raise RuntimeError(f'The simulator did not reset: {response.message}')


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the rotation matrix of a unit quaternion."""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_matrix(entry: dict) -> np.ndarray:
    """Return a task.yaml pose (xyz in m, rpy in degrees about the fixed axes) as 4 x 4."""
    r, p, y = np.radians(entry.get('rpy', [0.0, 0.0, 0.0]))
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = entry.get('xyz', [0.0, 0.0, 0.0])
    return T


def run_policy(expert: Expert, args: argparse.Namespace, spec: dict) -> tuple[bool, str]:
    """Run one episode of the stage with the policy; return success and the part's error."""
    stage = next(s for s in spec['stages'] if s['name'] == args.stage)
    part = stage['part']
    goal = spec['scene']['goal'][part]
    mate = goal['in']
    support = spec['scene']['start'][part].get('in')
    near = [x for x in (part, mate, support, 'table') if x]
    pairs = [(link, x) for link in GRIPPER_LINKS for x in near]
    pairs += [(part, x) for x in (mate, support) if x]
    pairs += [tuple(pair) for pair in stage.get('contacts', [])]
    expert.allow_contacts(pairs, True)
    try:
        command = [
            *LEROBOT,
            'rollout',
            '--strategy.type=base',
            f'--policy.path={args.policy}',
            '--robot.type=piper_ros',
            f'--robot.images={args.images}',
            f'--robot.effort={str(args.effort).lower()}',
            f'--task={args.task}',
            f'--duration={args.rollout_s}',
            '--fps=15',
        ]
        env = {**os.environ, 'HF_HUB_OFFLINE': '1', 'PYTHONPATH': str(LEROBOT_DIR)}
        subprocess.run(
            command,
            cwd=LEROBOT_DIR,
            env=env,
            timeout=args.rollout_s + 120,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    finally:
        expert.allow_contacts(pairs, False)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        rclpy.spin_once(expert.node, timeout_sec=0.1)
    if part not in expert.poses or mate not in expert.poses:
        return False, 'no ground truth'
    want = expert.poses[mate] @ pose_matrix(goal)
    have = expert.poses[part]
    error_m = float(np.linalg.norm(have[:3, 3] - want[:3, 3]))
    tilt = math.degrees(math.acos(float(np.clip(want[:3, 2] @ have[:3, 2], -1.0, 1.0))))
    return error_m < 0.003 and tilt < 20.0, f'{1e3 * error_m:.1f} mm, tilt {tilt:.0f} deg'


def main() -> int:
    """Run the episodes; return 0 when every one ran, kept or not."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--stage', required=True, help="The stage, a name in the task's stages")
    parser.add_argument('--task', required=True, help="The episodes' task text, for the dataset")
    parser.add_argument('--episodes', type=int, default=50, help='Episodes to run')
    parser.add_argument(
        '--poses', default='ready,view_left,view_right', help='SRDF poses an episode starts from'
    )
    parser.add_argument('--seed', type=int, default=0, help='Seed of the start poses')
    parser.add_argument('--stage-limit-s', type=float, default=600.0, help='Longest stage, s')
    parser.add_argument('--settle-s', type=float, default=1.5, help='Rest after a reset, s')
    parser.add_argument('--policy', help='A trained LeRobot policy (its pretrained_model) to run')
    parser.add_argument('--rollout-s', type=int, default=60, help="A policy episode's length, s")
    parser.add_argument('--task-file', default='/piper-stack/tasks/assembly/task.yaml')
    parser.add_argument('--images', default='wrist:480x640,overhead:360x640')
    parser.add_argument('--effort', action='store_true', help='The policy observes efforts')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('expert_episodes')
    expert = Expert(node, timeout_s=30.0)
    log = node.get_logger()
    rng = random.Random(args.seed)
    poses = args.poses.split(',')
    spec = yaml.safe_load(Path(args.task_file).read_text())
    kept = 0
    try:
        for n in range(args.episodes):
            pose = rng.choice(poses)
            moved = expert.run(expert.pose, MoveArmToNamedPose.Goal(name=pose), 120.0)
            if moved is None or not moved.success:
                log.warning(f'Episode {n}: the arm did not reach {pose}; skipped')
                continue
            expert.reset_before(args.stage)
            time.sleep(args.settle_s)
            expert.mark(f'record:{n}:{args.task}')
            started = time.monotonic()
            if args.policy:
                success, reason = run_policy(expert, args, spec)
            else:
                result = expert.run(
                    expert.stage, RunStage.Goal(stage=args.stage), args.stage_limit_s
                )
                success = result is not None and result.success
                reason = 'done' if success else (result.message if result else 'no result')
            expert.mark(f'{"keep" if success else "discard"}:{n}')
            kept += success
            log.info(
                f'Episode {n} from {pose}: {"kept" if success else "discarded"} after '
                f'{time.monotonic() - started:.0f} s ({reason}); {kept} kept of {n + 1}'
            )
            # The recorder reads the verdict at its own rate: hold it a moment.
            time.sleep(1.0)
        expert.mark('end')
        time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

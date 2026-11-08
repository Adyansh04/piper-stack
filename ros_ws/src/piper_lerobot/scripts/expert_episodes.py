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

Python: it sequences ROS services and actions a few times a minute (data generation).
"""

from __future__ import annotations

import argparse
import random
import sys
import time

import rclpy
from piper_msgs.action import MoveArmToNamedPose, RunStage
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

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
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('expert_episodes')
    expert = Expert(node, timeout_s=30.0)
    log = node.get_logger()
    rng = random.Random(args.seed)
    poses = args.poses.split(',')
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
            result = expert.run(expert.stage, RunStage.Goal(stage=args.stage), args.stage_limit_s)
            success = result is not None and result.success
            expert.mark(f'{"keep" if success else "discard"}:{n}')
            kept += success
            reason = 'done' if success else (result.message if result else 'no result in time')
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

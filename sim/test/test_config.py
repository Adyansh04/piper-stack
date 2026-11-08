"""Settings that two packages must agree on (no Isaac Sim needed)."""

from __future__ import annotations

from pathlib import Path

import yaml

ROS = Path(__file__).resolve().parents[2] / 'ros_ws' / 'src'


def test_the_policy_runner_keeps_its_targets_inside_servos_joint_margin() -> None:
    servo = yaml.safe_load((ROS / 'piper_moveit_config/config/servo.yaml').read_text())
    runner = yaml.safe_load((ROS / 'piper_controllers/config/policy_runner.yaml').read_text())
    margins = servo['servo_node']['ros__parameters']['moveit_servo']['joint_limit_margins']
    # Servo halts the arm once a joint is past its margin: the policy must never ask for that.
    assert runner['policy_runner']['ros__parameters']['limit_margin_rad'] > max(margins)

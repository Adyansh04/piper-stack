"""Starts the reach policy runner: a trained policy moving the arm through MoveIt Servo.

Needs MoveIt Servo running (bringup's moveit:=true servo:=true) and the robot's state on
/joint_states. The runner starts inactive; ~/set_active switches it on.

    ros2 launch piper_controllers policy_runner.launch.py \
        model_path:=/piper-stack/data/policies/reach/policy.onnx use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Return the runner with its parameters, the robot description for the joint limits."""
    description = Command(
        [
            'xacro ',
            PathJoinSubstitution(
                [FindPackageShare('piper_description'), 'urdf', 'piper.urdf.xacro']
            ),
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model_path',
                description='The trained policy: policy.onnx, with its policy.onnx.data beside it.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='true: on the simulator clock (/clock), as the rest of the stack.',
            ),
            Node(
                package='piper_controllers',
                executable='policy_runner',
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare('piper_controllers'), 'config', 'policy_runner.yaml']
                    ),
                    {
                        'model_path': LaunchConfiguration('model_path'),
                        'robot_description': ParameterValue(description, value_type=str),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    },
                ],
                output='screen',
            ),
        ]
    )

"""LeRobot's bridge to the stack: lerobot_bridge, serving LeRobot's plugins over ZMQ.

Needs the stack with MoveIt Servo (bringup.launch.py motion:=true moveit:=true servo:=true):
    ros2 launch piper_lerobot lerobot_bridge.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='true: follow /clock from the simulator.',
            ),
            Node(
                package='piper_lerobot',
                executable='lerobot_bridge',
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare('piper_lerobot'), 'config', 'lerobot_bridge.yaml']
                    ),
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool
                        )
                    },
                ],
                output='screen',
            ),
        ]
    )

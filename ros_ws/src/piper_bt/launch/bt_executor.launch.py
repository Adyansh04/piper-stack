"""Start the behaviour tree executor with its parameter file.

ros2 launch piper_bt bt_executor.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and the executor node."""
    default_config = PathJoinSubstitution(
        [FindPackageShare('piper_bt'), 'config', 'bt_executor.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='ROS namespace of the executor: run_tree and every skill server its '
                'leaves call are relative to it. Empty: the root namespace, where piper_skills '
                'serves.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='true: take the time from /clock, as in the simulator; false: the wall '
                'clock, as on the robot.',
            ),
            DeclareLaunchArgument(
                'config',
                default_value=default_config,
                description="Path of the executor's parameter YAML. Default: this package's "
                'config/bt_executor.yaml.',
            ),
            Node(
                package='piper_bt',
                executable='bt_executor',
                name='bt_executor',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    LaunchConfiguration('config'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                output='screen',
            ),
        ]
    )

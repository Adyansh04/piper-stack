"""Start the SAM 3.1 segmentation service with its parameter file.

ros2 launch piper_perception sam3_segmentation.launch.py namespace:=piperAA use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and the segmentation node."""
    default_config = PathJoinSubstitution(
        [FindPackageShare('piper_perception'), 'config', 'sam3_segmentation.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='ROS namespace of the node; its service and topics are relative to '
                'it. Empty: the root namespace, where the skills look for it.',
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
                description="Path of the parameter YAML. Default: this package's "
                'config/sam3_segmentation.yaml.',
            ),
            Node(
                package='piper_perception',
                executable='sam3_segmentation_node',
                name='sam3_segmentation',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    LaunchConfiguration('config'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                respawn=True,
                respawn_delay=5.0,
                output='screen',
            ),
        ]
    )

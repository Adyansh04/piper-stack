"""Start the GraspGenX server and the compute_grasps service in front of it.

ros2 launch piper_perception grasp_generation.launch.py
ros2 launch piper_perception grasp_generation.launch.py start_server:=false  # server runs elsewhere
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments, the server process and the node."""
    share = FindPackageShare('piper_perception')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='ROS namespace of the node; its service is relative to it. Empty: the '
                'root namespace, where the skills look for it.',
            ),
            DeclareLaunchArgument(
                'start_server',
                default_value='true',
                choices=['true', 'false'],
                description='true: start the GraspGenX server here; false: it is already running '
                "at the node's server_endpoint.",
            ),
            DeclareLaunchArgument(
                'python',
                default_value='/opt/graspgenx-venv/bin/python',
                description="Python of the GraspGenX venv. Default: the dev image's.",
            ),
            DeclareLaunchArgument(
                'server_config',
                default_value=PathJoinSubstitution([share, 'config', 'graspgenx_server.yaml']),
                description="The server's parameter YAML. Default: this package's "
                'config/graspgenx_server.yaml.',
            ),
            DeclareLaunchArgument(
                'config',
                default_value=PathJoinSubstitution([share, 'config', 'grasp_generation.yaml']),
                description="The node's parameter YAML. Default: this package's "
                'config/grasp_generation.yaml.',
            ),
            # The server keeps no state between requests, and exits after a CUDA error so that
            # a fresh process takes over.
            ExecuteProcess(
                name='graspgenx_server',
                cmd=[
                    LaunchConfiguration('python'),
                    PathJoinSubstitution([share, 'graspgenx', 'graspgenx_server.py']),
                    '--params',
                    LaunchConfiguration('server_config'),
                ],
                condition=IfCondition(LaunchConfiguration('start_server')),
                respawn=True,
                respawn_delay=5.0,
                output='screen',
            ),
            Node(
                package='piper_perception',
                executable='grasp_generation_node',
                name='grasp_generation',
                namespace=LaunchConfiguration('namespace'),
                parameters=[LaunchConfiguration('config')],
                respawn=True,
                respawn_delay=5.0,
                output='screen',
            ),
        ]
    )

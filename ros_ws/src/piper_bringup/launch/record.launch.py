"""Record a profile of the stack's topics into a new MCAP bag under bag_dir.

ros2 launch piper_bringup record.launch.py profile:=arm
ros2 launch piper_bringup record.launch.py profile:=mission use_sim_time:=true

Ctrl-C stops the recording and closes the bag. The profiles are config/record/<profile>.yaml.
"""

import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def recorder(context):
    """The recorder of one profile, writing to <bag_dir>/<profile>_<date>_<time>."""
    profile = LaunchConfiguration('profile').perform(context)
    bag = '{}/{}_{}'.format(
        LaunchConfiguration('bag_dir').perform(context), profile, time.strftime('%Y%m%d_%H%M%S')
    )
    return [
        Node(
            package='rosbag2_transport',
            executable='recorder',
            name='recorder',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('piper_bringup'), 'config', 'record', f'{profile}.yaml']
                ),
                {'storage.uri': bag, 'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and the recorder."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'profile',
                default_value='arm',
                choices=['arm', 'mission', 'camera'],
                description='arm: joints, commands and loop timing; mission: arm plus the '
                "behaviour tree's status and the skills' actions; camera: the wrist camera's "
                'images.',
            ),
            DeclareLaunchArgument(
                'bag_dir',
                default_value='/piper-stack/data/bags',
                description="Where the bag's directory goes (the checkout's data/, ignored by "
                'git).',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='true: stamp the messages with /clock, as in the simulator; false: '
                'the wall clock, as on the robot.',
            ),
            OpaqueFunction(function=recorder),
        ]
    )

"""The arm's contact and grasp estimates: ~/tcp_wrench and ~/grasped of state_estimator.

ros2 launch piper_estimation state_estimator.launch.py
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
                package='piper_estimation',
                executable='state_estimator',
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare('piper_estimation'), 'config', 'state_estimator.yaml']
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

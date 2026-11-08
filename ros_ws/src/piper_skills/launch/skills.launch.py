"""Start the manipulation skills with their parameters and MoveIt's description of the arm.

Needs the stack (piper_bringup with moveit:=true), segment_objects and compute_grasps:
    ros2 launch piper_skills skills.launch.py
    ros2 launch piper_skills skills.launch.py use_sim_time:=true   # against Isaac Sim
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and the skills node."""
    xacro = os.path.join(
        get_package_share_directory('piper_description'), 'urdf', 'piper.urdf.xacro'
    )
    # The same description move_group runs on: the SRDF's named poses and the kinematics for IK.
    moveit = (
        MoveItConfigsBuilder('piper', package_name='piper_moveit_config')
        .robot_description(file_path=xacro)
        .robot_description_semantic(file_path='config/piper.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='true: follow /clock from the simulator; false: the wall clock.',
            ),
            DeclareLaunchArgument(
                'config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('piper_skills'), 'config', 'skills.yaml']
                ),
                description="The skills' parameter YAML. Default: this package's config/skills.yaml.",
            ),
            Node(
                package='piper_skills',
                executable='skills_node',
                name='piper_skills',
                parameters=[
                    moveit.robot_description,
                    moveit.robot_description_semantic,
                    moveit.robot_description_kinematics,
                    moveit.joint_limits,
                    LaunchConfiguration('config'),
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

"""MoveIt 2 for the Piper: move_group (OMPL and Pilz, pick_ik), MoveIt Servo, and RViz.

Needs robot_state_publisher and the controllers up; piper_bringup starts this with moveit:=true:
    ros2 launch piper_bringup bringup.launch.py hardware:=mock motion:=true moveit:=true rviz:=true
On its own, next to a running bringup:
    ros2 launch piper_moveit_config moveit.launch.py servo:=true rviz:=true
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('piper_moveit_config')
    xacro = os.path.join(
        get_package_share_directory('piper_description'), 'urdf', 'piper.urdf.xacro'
    )
    config = (
        MoveItConfigsBuilder('piper', package_name='piper_moveit_config')
        .robot_description(file_path=xacro)
        .robot_description_semantic(file_path='config/piper.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner'], default_planning_pipeline='ompl'
        )
        # robot_state_publisher publishes the description; move_group adds the semantic one, so
        # clients (skills, MoveGroupInterface) need neither as a parameter.
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    sim_time = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    }
    description = [
        config.robot_description,
        config.robot_description_semantic,
        config.robot_description_kinematics,
        config.joint_limits,
    ]
    with open(os.path.join(share, 'config', 'servo_kinematics.yaml')) as f:
        servo_kinematics = {'robot_description_kinematics': yaml.safe_load(f)}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'servo',
                default_value='false',
                choices=['true', 'false'],
                description='true: also start MoveIt Servo for jogging (keyboard_jog).',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                choices=['true', 'false'],
                description='true: open RViz with the MotionPlanning panel.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='true: follow /clock from the simulator.',
            ),
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                parameters=[config.to_dict(), sim_time],
                output='screen',
            ),
            Node(
                package='moveit_servo',
                executable='servo_node',
                name='servo_node',
                parameters=[
                    os.path.join(share, 'config', 'servo.yaml'),
                    config.robot_description,
                    config.robot_description_semantic,
                    servo_kinematics,
                    config.joint_limits,
                    sim_time,
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('servo')),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(share, 'config', 'moveit.rviz')],
                parameters=[*description, config.planning_pipelines, sim_time],
                output='log',
                condition=IfCondition(LaunchConfiguration('rviz')),
            ),
        ]
    )

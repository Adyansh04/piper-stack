"""The Piper's ros2_control stack: robot_state_publisher, the controller_manager, the controllers.

Read-only on the real arm (motors untouched, nothing sent):
    ros2 launch piper_bringup bringup.launch.py hardware:=can rviz:=true
Commanding the real arm, firmware speed at 10 %:
    ros2 launch piper_bringup bringup.launch.py hardware:=can enable:=true motion:=true
Without the arm:
    ros2 launch piper_bringup bringup.launch.py hardware:=mock motion:=true rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = LaunchConfiguration('hardware')
    enable = LaunchConfiguration('enable')
    motion = LaunchConfiguration('motion')
    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                PathJoinSubstitution(
                    [FindPackageShare('piper_description'), 'urdf', 'piper.urdf.xacro']
                ),
                ' hardware:=',
                hardware,
                ' can_interface:=',
                LaunchConfiguration('can_interface'),
                ' enable_on_activate:=',
                enable,
                ' speed_percent:=',
                LaunchConfiguration('speed_percent'),
            ]
        ),
        value_type=str,
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare('piper_bringup'), 'config', 'controllers.yaml']
    )

    def spawner(name, condition=None):
        return Node(
            package='controller_manager',
            executable='spawner',
            # Controllers take their parameters from the spawner, not from the controller_manager.
            arguments=[
                name,
                '--controller-manager',
                '/controller_manager',
                '--param-file',
                controllers,
            ],
            condition=condition,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'hardware',
                default_value='mock',
                choices=['mock', 'can'],
                description='mock: ros2_control mock, no arm. can: the real arm over CAN.',
            ),
            DeclareLaunchArgument(
                'can_interface',
                default_value='can_follower',
                description='CAN interface of the arm, up at 1 Mbit/s (hardware:=can).',
            ),
            DeclareLaunchArgument(
                'enable',
                default_value='false',
                choices=['true', 'false'],
                description='hardware:=can only. true: enable the motors and send commands. false: '
                'read-only, nothing is sent to the arm.',
            ),
            DeclareLaunchArgument(
                'speed_percent',
                default_value='10',
                description='Firmware speed scaling for MOVE J, 1-100 percent of 3 rad/s.',
            ),
            DeclareLaunchArgument(
                'motion',
                default_value='false',
                choices=['true', 'false'],
                description='true: also start the arm and gripper controllers. '
                'false: joint states only.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                choices=['true', 'false'],
                description='true: open RViz on this display.',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[controllers],
                remappings=[('~/robot_description', '/robot_description')],
                output='screen',
            ),
            spawner('joint_state_broadcaster'),
            spawner('arm_controller', IfCondition(motion)),
            spawner('gripper_controller', IfCondition(motion)),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d',
                    PathJoinSubstitution([FindPackageShare('piper_bringup'), 'rviz', 'piper.rviz']),
                ],
                condition=IfCondition(rviz),
            ),
        ]
    )

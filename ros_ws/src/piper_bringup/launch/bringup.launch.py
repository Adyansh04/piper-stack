"""The Piper's stack: robot_state_publisher, the controller_manager, the controllers, MoveIt.

Read-only on the real arm (motors untouched, nothing sent):
    ros2 launch piper_bringup bringup.launch.py hardware:=can rviz:=true
Commanding the real arm, firmware speed at 10 %:
    ros2 launch piper_bringup bringup.launch.py hardware:=can enable:=true motion:=true
Without the arm, planning and jogging in RViz:
    ros2 launch piper_bringup bringup.launch.py hardware:=mock motion:=true moveit:=true rviz:=true
Against Isaac Sim (sim/sim.py running), on its clock:
    ros2 launch piper_bringup bringup.launch.py hardware:=isaac motion:=true moveit:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    EqualsSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = LaunchConfiguration('hardware')
    motion = LaunchConfiguration('motion')
    moveit = LaunchConfiguration('moveit')
    rviz = LaunchConfiguration('rviz')
    # The simulator's clock drives every node of the stack, or none.
    use_sim_time = EqualsSubstitution(hardware, 'isaac')

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
                LaunchConfiguration('enable'),
                ' speed_percent:=',
                LaunchConfiguration('speed_percent'),
                ' motion_mode:=',
                LaunchConfiguration('motion_mode'),
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
                choices=['mock', 'can', 'isaac'],
                description='mock: ros2_control mock, no arm. can: the real arm over CAN. isaac: '
                'the arm in Isaac Sim (sim/sim.py), over its joint topics and on its clock.',
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
                'motion_mode',
                default_value='joint',
                choices=['joint', 'cpv', 'mit'],
                description='hardware:=can only. joint: MOVE J, the firmware interpolates each '
                'target. cpv: MOVE CPV streaming. mit: per-joint impedance (gains in the URDF).',
            ),
            DeclareLaunchArgument(
                'motion',
                default_value='false',
                choices=['true', 'false'],
                description='true: also start the arm and gripper controllers. '
                'false: joint states only.',
            ),
            DeclareLaunchArgument(
                'moveit',
                default_value='false',
                choices=['true', 'false'],
                description='true: also start MoveIt (move_group); needs motion:=true to move.',
            ),
            DeclareLaunchArgument(
                'servo',
                default_value='false',
                choices=['true', 'false'],
                description='moveit:=true only. true: also start MoveIt Servo for jogging.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='false',
                choices=['true', 'false'],
                description='true: open RViz here, with the MoveIt panel if moveit:=true.',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[controllers, {'use_sim_time': use_sim_time}],
                remappings=[('~/robot_description', '/robot_description')],
                output='screen',
            ),
            spawner('joint_state_broadcaster'),
            spawner('arm_controller', IfCondition(motion)),
            spawner('gripper_controller', IfCondition(motion)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('piper_moveit_config'), 'launch', 'moveit.launch.py']
                    )
                ),
                launch_arguments={
                    'servo': LaunchConfiguration('servo'),
                    'rviz': rviz,
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(moveit),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d',
                    PathJoinSubstitution([FindPackageShare('piper_bringup'), 'rviz', 'piper.rviz']),
                ],
                condition=IfCondition(AndSubstitution(rviz, NotSubstitution(moveit))),
            ),
        ]
    )

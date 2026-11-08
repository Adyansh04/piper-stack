"""Start task_scene, which keeps MoveIt's planning scene in step with a task's parts.

Needs move_group (piper_bringup with moveit:=true) and the parts' poses (Isaac Sim's sim/objects):
    ros2 launch piper_skills task_scene.launch.py \
        task_file:=/piper-stack/tasks/assembly/task.yaml use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and the task_scene node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'task_file',
                description="The task's task.yaml (tasks/<task>/task.yaml): its parts and meshes.",
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='true: on the simulator clock (/clock), as the rest of the stack.',
            ),
            Node(
                package='piper_skills',
                executable='task_scene',
                name='task_scene',
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare('piper_skills'), 'config', 'task_scene.yaml']
                    ),
                    {
                        'task_file': LaunchConfiguration('task_file'),
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool
                        ),
                    },
                ],
                output='screen',
            ),
        ]
    )

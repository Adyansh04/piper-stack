"""The wrist camera (Percipio GM461-E1) through Percipio's driver, with an optional RViz view.

    ros2 launch piper_bringup camera.launch.py device_ip:=192.168.10.200 rviz:=true
    ros2 launch piper_bringup camera.launch.py color_resolution:=1280x960 depth_resolution:=1280x800

Modes: colour 1280x960, 640x480, 320x240 (JPEG); depth 1280x800, 640x400, 320x200; 8-30 fps.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Forwarded to Percipio's launch file as they are: (name, default, description).
CAMERA_ARGS = [
    ('device_ip', '192.168.10.200', 'Camera IP; empty opens the first camera found.'),
    ('color_resolution', '640x480', 'Colour mode: 1280x960, 640x480 or 320x240.'),
    ('depth_resolution', '640x400', 'Depth mode: 1280x800, 640x400 or 320x200.'),
    ('frame_rate_control', 'true', 'true: cap the rate at frame_rate. false: as fast as it streams.'),
    ('frame_rate', '15.0', 'Frame rate cap in fps, 8 to 30, with frame_rate_control true.'),
    ('depth_registration_enable', 'true', 'true: depth aligned to the colour camera.'),
    ('color_point_cloud_enable', 'true', 'true: publish the coloured point cloud.'),
    ('depth_speckle_filter', 'true', 'true: drop small isolated depth blobs.'),
    ('depth_time_domain_filter', 'false', 'true: average depth over frames (lags a moving wrist).'),
]


def generate_launch_description():
    percipio = PathJoinSubstitution(
        [FindPackageShare('percipio_camera'), 'launch', 'percipio_camera.launch.py'])
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in CAMERA_ARGS],
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='true: open RViz with colour, depth and the coloured cloud.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(percipio),
            launch_arguments={name: LaunchConfiguration(name) for name, _, _ in CAMERA_ARGS}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('piper_bringup'), 'rviz', 'camera.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])

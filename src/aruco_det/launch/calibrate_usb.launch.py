"""Optional interactive chessboard calibration; requires camera_calibration."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path = Path(get_package_share_directory('aruco_det')) / 'launch/usb_aruco.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('board_size', default_value='8x6'),
        DeclareLaunchArgument('square_size', default_value='0.025'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                 launch_arguments={'start_detector': 'false'}.items()),
        Node(package='camera_calibration', executable='cameracalibrator',
             name='camera_calibration', output='screen',
             arguments=['--size', LaunchConfiguration('board_size'),
                        '--square', LaunchConfiguration('square_size')],
             remappings=[('image', '/usb_cam/image_raw'), ('camera', '/usb_cam')]),
    ])

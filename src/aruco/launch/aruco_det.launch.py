from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw/compressed',
        description='输入相机压缩图像话题'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/prometheus/camera/rgb/image_aruco_det',
        description='输出检测可视化图像话题'
    )

    dictionary_type_arg = DeclareLaunchArgument(
        'dictionary_type',
        default_value='10',
        description='ArUco 字典枚举值 (10 对应 DICT_6X6_250)'
    )

    marker_length_arg = DeclareLaunchArgument(
        'marker_length',
        default_value='0.16',
        description='ArUco 二维码实际物理边长（单位：米）'
    )

    # 直接在这里修改你的相机内参 [fx, 0, cx, 0, fy, cy, 0, 0, 1] 和畸变系数 [k1, k2, p1, p2, k3]
    camera_matrix = [
        600.0,   0.0, 320.0,
          0.0, 600.0, 240.0,
          0.0,   0.0,   1.0
    ]
    distortion_coefficients = [0.0, 0.0, 0.0, 0.0, 0.0]

    aruco_node = Node(
        package='aruco',
        executable='aruco_det_git_xy',
        name='aruco_det_git',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'dictionary_type': LaunchConfiguration('dictionary_type'),
            'marker_length': LaunchConfiguration('marker_length'),
            'camera_frame': 'camera',
            'camera_matrix': camera_matrix,
            'distortion_coefficients': distortion_coefficients,
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        output_topic_arg,
        dictionary_type_arg,
        marker_length_arg,
        aruco_node
    ])
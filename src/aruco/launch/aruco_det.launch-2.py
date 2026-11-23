from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw/compressed',
        description='输入的相机图像话题名称'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/prometheus/camera/rgb/image_aruco_det',
        description='发布的检测结果可视化图像话题名称'
    )

    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/camera_link_state',
        description='发布的相机坐标系下二维码位姿话题名称'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera',
        description='相机坐标系 Frame ID'
    )

    dictionary_type_arg = DeclareLaunchArgument(
        'dictionary_type',
        default_value='10',
        description='ArUco 字典编号 (默认 10 对应 DICT_6X6_250)'
    )

    marker_length_arg = DeclareLaunchArgument(
        'marker_length',
        default_value='0.16',
        description='ArUco 二维码实际物理边长（单位：米）'
    )

    # 若需沿用原有的 YAML 文件加载内参，可将路径填入此处（非空时优先于下方 camera_matrix）
    camera_parameters_yaml_arg = DeclareLaunchArgument(
        'camera_parameters_yaml',
        default_value='',
        description='可选：OpenCV 格式相机内参 YAML 文件路径'
    )

    # 直接在 Launch 中修改相机内参 [fx, 0, cx, 0, fy, cy, 0, 0, 1] 与畸变参数 [k1, k2, p1, p2, k3]
    camera_matrix = [
        294.0025685593909,   0.0, 327.8364832091295,
          0.0, 296.1201247428658, 237.8116466983579,
          0.0,   0.0,   1.0
    ]
    dist_coeffs = [-0.021385581716086424, -0.008718759329317297, 0.002012884309469192, 0.00041578810898600663, 0.0]

    aruco_det_node = Node(
        package='aruco',
        executable='aruco_det_git_xy',
        name='aruco_det_git',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'dictionary_type': LaunchConfiguration('dictionary_type'),
            'marker_length': LaunchConfiguration('marker_length'),
            'camera_parameters_yaml': LaunchConfiguration('camera_parameters_yaml'),
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs,
            # 如需启用原代码 ID=19 (0.404m) 与 ID=43 (0.044m) 远近双码切换，改为 True
            'enable_dual_marker_mode': False,
            'large_marker_id': 19,
            'large_marker_length': 0.404,
            'small_marker_id': 43,
            'small_marker_length': 0.044,
            'switch_distance_z': 5.0,
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        output_topic_arg,
        pose_topic_arg,
        camera_frame_arg,
        dictionary_type_arg,
        marker_length_arg,
        camera_parameters_yaml_arg,
        aruco_det_node
    ])
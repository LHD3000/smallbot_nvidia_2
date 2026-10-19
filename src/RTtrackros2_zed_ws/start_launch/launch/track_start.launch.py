from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_face_image = LaunchConfiguration("target_face_image")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    face_similarity_threshold = LaunchConfiguration("face_similarity_threshold")
    visualize = LaunchConfiguration("visualize")
    save_vid = LaunchConfiguration("save_vid")
    camera_fx = LaunchConfiguration("camera_fx")
    camera_cx = LaunchConfiguration("camera_cx")

    balance_node = Node(
        package="balance_ros",
        executable="balance_track_node",
        name="balance_track_node",
        output="screen",
    )

    yolo_node = Node(
        package="rttrack",
        executable="yolotrack_trt",
        name="yolotrack_trt",
        output="screen",
        parameters=[
            {"target_face_image": target_face_image},
            {"image_topic": image_topic},
            {"depth_topic": depth_topic},
            {"face_similarity_threshold": face_similarity_threshold},
            {"visualize": visualize},
            {"save_vid": save_vid},
        ],
    )

    angle_node = Node(
        package="chasis_cpp",
        executable="angle_calculator_node",
        name="angle_calculator_node",
        output="screen",
        parameters=[
            {"camera_fx": camera_fx},
            {"camera_cx": camera_cx},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_face_image",
                default_value="/home/robot/dwrobot/RTtrackros2_zed_ws/src/rttrack/config/jk1.png",
                description="Path to the target face image for yolotrack_trt.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/zed/zed_node/left/color/rect/image",
                description="Input image topic for yolotrack_trt.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/zed/zed_node/depth/depth_registered",
                description="Input depth image topic for yolotrack_trt.",
            ),
            DeclareLaunchArgument(
                "face_similarity_threshold",
                default_value="0.5",
                description="Face match threshold for yolotrack_trt.",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="false",
                description="Enable yolotrack_trt visualization.",
            ),
            DeclareLaunchArgument(
                "save_vid",
                default_value="false",
                description="Enable saving video in $HOME.",
            ),
            DeclareLaunchArgument(
                "camera_fx",
                default_value="266.2",
                description="Camera focal length fx for angle_calculator_node.",
            ),
            DeclareLaunchArgument(
                "camera_cx",
                default_value="320.6",
                description="Camera principal point cx for angle_calculator_node.",
            ),
            balance_node,
            TimerAction(
                period=1.0,
                actions=[
                    yolo_node
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    angle_node
                ],
            ),
        ]
    )

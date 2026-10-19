from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    target_distance = LaunchConfiguration("target_distance")
    kp_distance = LaunchConfiguration("kp_distance")
    ki_distance = LaunchConfiguration("ki_distance")
    kd_distance = LaunchConfiguration("kd_distance")
    kp_angle = LaunchConfiguration("kp_angle")
    ki_angle = LaunchConfiguration("ki_angle")
    kd_angle = LaunchConfiguration("kd_angle")
    max_linear = LaunchConfiguration("max_linear")
    max_angular = LaunchConfiguration("max_angular")
    max_angular_accel = LaunchConfiguration("max_angular_accel")
    filter_alpha = LaunchConfiguration("filter_alpha")
    dead_zone_deg = LaunchConfiguration("dead_zone_deg")
    adaptive_deg = LaunchConfiguration("adaptive_deg")
    adaptive_dim = LaunchConfiguration("adaptive_dim")
    smooth_factor = LaunchConfiguration("smooth_factor")
    singular_thresh = LaunchConfiguration("singular_thresh")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_distance",
                default_value="1.5",
                description="Target distance (meters) for pid_movebase_node.",
            ),
            DeclareLaunchArgument(
                "kp_distance",
                default_value="1.15",
                description="P gain for distance control.",
            ),
            DeclareLaunchArgument(
                "ki_distance",
                default_value="0.0",
                description="I gain for distance control.",
            ),
            DeclareLaunchArgument(
                "kd_distance",
                default_value="0.12",
                description="D gain for distance control.",
            ),
            DeclareLaunchArgument(
                "kp_angle",
                default_value="1.10",
                description="P gain for angle control.",
            ),
            DeclareLaunchArgument(
                "ki_angle",
                default_value="0.0",
                description="I gain for angle control.",
            ),
            DeclareLaunchArgument(
                "kd_angle",
                default_value="0.14",
                description="D gain for angle control.",
            ),
            DeclareLaunchArgument(
                "max_linear",
                default_value="0.7",
                description="Max linear velocity (m/s).",
            ),
            DeclareLaunchArgument(
                "max_angular",
                default_value="1.2",
                description="Max angular velocity (rad/s).",
            ),
            DeclareLaunchArgument(
                "max_angular_accel",
                default_value="1.5",
                description="Max angular acceleration (rad/s^2).",
            ),
            DeclareLaunchArgument(
                "filter_alpha",
                default_value="0.35",
                description="Low-pass filter alpha for distance/angle rates.",
            ),
            DeclareLaunchArgument(
                "dead_zone_deg",
                default_value="0.3",
                description="Dead zone for angle in degrees.",
            ),
            DeclareLaunchArgument(
                "adaptive_deg",
                default_value="5.0",
                description="Adaptive gain threshold for angle (deg).",
            ),
            DeclareLaunchArgument(
                "adaptive_dim",
                default_value="0.1",
                description="Adaptive gain threshold for distance (m).",
            ),
            DeclareLaunchArgument(
                "smooth_factor",
                default_value="0.3",
                description="Smooth saturation factor.",
            ),
            DeclareLaunchArgument(
                "singular_thresh",
                default_value="0.15",
                description="Singular threshold for cos(phi).",
            ),
            Node(
                package="chasis_cpp",
                executable="pid_movebase_node",
                name="pid_movebase_node",
                output="screen",
                parameters=[
                    {
                        "target_distance": ParameterValue(target_distance, value_type=float),
                        "kp_distance": ParameterValue(kp_distance, value_type=float),
                        "ki_distance": ParameterValue(ki_distance, value_type=float),
                        "kd_distance": ParameterValue(kd_distance, value_type=float),
                        "kp_angle": ParameterValue(kp_angle, value_type=float),
                        "ki_angle": ParameterValue(ki_angle, value_type=float),
                        "kd_angle": ParameterValue(kd_angle, value_type=float),
                        "max_linear": ParameterValue(max_linear, value_type=float),
                        "max_angular": ParameterValue(max_angular, value_type=float),
                        "max_angular_accel": ParameterValue(max_angular_accel, value_type=float),
                        "filter_alpha": ParameterValue(filter_alpha, value_type=float),
                        "dead_zone_deg": ParameterValue(dead_zone_deg, value_type=float),
                        "adaptive_deg": ParameterValue(adaptive_deg, value_type=float),
                        "adaptive_dim": ParameterValue(adaptive_dim, value_type=float),
                        "smooth_factor": ParameterValue(smooth_factor, value_type=float),
                        "singular_thresh": ParameterValue(singular_thresh, value_type=float),
                    }
                ],
            ),
        ]
    )

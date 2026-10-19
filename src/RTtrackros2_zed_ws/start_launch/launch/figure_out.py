import math
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushROSNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction

import numpy as np


def generate_launch_description():

    plot1 = Node(
        package="toolbox",
        executable="plot_runtime",
        name="plot_runtime",
        output="screen",
        parameters=[
            {
                "topics": "torque,rotate_torque,target_angle,angle,target_gyro,gyro,target_gyroz,gyroz,target_speed,speed"
            },
            {"auto_start": True},
        ],
        namespace="robot1",
    )

    plot2 = Node(
        package="toolbox",
        executable="plot_runtime",
        name="plot_runtime",
        output="screen",
        parameters=[
            {
                "topics": "torque,rotate_torque,target_angle,angle,target_gyro,gyro,target_gyroz,gyroz,target_speed,speed"
            },
            {"auto_start": True},
        ],
        namespace="robot2",
    )

    return LaunchDescription(
        [
            plot1,
            # plot2,
        ]
    )

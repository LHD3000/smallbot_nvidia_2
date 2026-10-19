from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. 启动手柄底层驱动节点
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.1, # 设置摇杆死区，防止摇杆松动导致车乱飘
            }]
        ),

        # 2. 启动手柄指令转换节点
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[{
                'require_enable_button': False, # 强制要求按下保险键
                'enable_button': 4,            # 保险键设为 R1 (通常是5，也可能是4，视手柄而定)
                'axis_linear.x': 1,            # 前进后退映射到左摇杆上下
                'scale_linear.x': 0.8,         # 前进最大速度 0.8m/s (配合你代码里的系数)
                'axis_angular.yaw': 0,         # 左右转向映射到左摇杆左右
                'scale_angular.yaw': 1.0,      # 转向最大速度
            }],
            # 如果以后你的代码改了订阅的话题名，可以在这里重映射
            remappings=[
                ('/cmd_vel', '/main/cmd_vel')
            ]
        ),
        Node(
            package='balance_ros',
            executable='balance_track_node_v2',
            name='balance_controller',
            output='screen',
            parameters=[{
                'banlance_offset': -4.8,
                'gyroPID': [430.0, 0.0, 0.0],
                'anglePID': [8.3, 0.0, 1.4],
                'gyroZPID': [295, 0.0, 0.0],
                'speedPID': [15.0, 0.0, 0.5]
            }]
        )
     ])

      

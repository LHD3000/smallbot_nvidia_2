# aruco_det

ROS 2 Humble / Jazzy ArUco检测包，使用rclcpp、rosidl与ament_cmake。

完整操作见 [工作区README](../../README.md)。默认字典10、黑色标记边长0.08m、主ID0，launch默认加载原内参。

```bash
ros2 launch aruco_det usb_aruco.launch.py video_device:=/dev/video4
ros2 run aruco_det subscribe_xyz.py
```

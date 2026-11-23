
sudo ln -s /dev/ttyUSB1 /dev/imu_usb1
ros2 launch wit_ros2_imu rviz_and_imu.launch.py 
%新建终端
cd /home/nvidia/jeston/smallbot_ws
source install/setup.bash
ros2 run bbot_rc_receiver rc_node

%新建终端
cd /home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src
sh can_set.sh

%新建终端
source install/setup.bash
ros2 run balance_ros balance_track_node （主控）

sudo date -s "2026-10-28 22:00:00"
rm -rf build/balance_ros install/balance_ros
colcon build（编译）
sudo poweroff（关机）

cd /home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src
python3 raw_plot.py (画图)

视觉
cd ~/jeston/smallbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
接收相机话题：/image_raw/compressed
输出二维码检测话题：/prometheus/camera/rgb/image_aruco_det
检测结果话题（位姿）：/camera_link_state
可视化：ros2 run rqt_image_view rqt_image_view，左上角选择对应图像话题即可。
查看摄像头支持的分辨率和帧数v4l2-ctl -d /dev/video0 --list-formats-ext
cd ~/jeston/smallbot_ws
./start_vision.sh
python3 ~/jeston/smallbot_ws/src/aruco/src/vision_plot.py

python3 ~/capture_chess.py /usb_cam/image_raws
ros2 topic echo /aruco/camera_pose --field pose.pose.position
ros2 topic echo /camera_link_state --field twist.twist.linear.x
ros2 topic hz /camera_link_state

对接电机
cd /home/nvidia/jeston/smallbot_ws
source install/setup.bash
ros2 run dock_motor_controller dock_motor_controller


ros2 topic echo  /....  --once

sudo systemctl restart imu.service
sudo systemctl restart robot_manager.service

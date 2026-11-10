
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
ros2 launch aruco_det usb_aruco.launch.py video_device:=/dev/video0 view:=false


对接电机
cd /home/nvidia/jeston/smallbot_ws
source install/setup.bash
ros2 run dock_motor_controller dock_motor_controller


ros2 topic echo  /....  --once

sudo systemctl restart imu.service
sudo systemctl restart robot_manager.service
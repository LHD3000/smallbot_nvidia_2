cd /home/nvidia/jeston/smallbot_ws
source install/setup.bash
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

colcon build（编译）
sudo poweroff（关机）


python3 raw_plot.py (画图)


对接电机
cd /sys/class/pwm/pwmchip3
echo 0 | sudo tee export
ls -l pwm0
cd pwm0
echo 50000 | sudo tee period
echo 25000 | sudo tee duty_cycle
echo 1 | sudo tee enable
sudo cat /sys/kernel/debug/pwm
启动
echo 50000 | sudo tee /sys/class/pwm/pwmchip3/pwm0/period
echo 40000 | sudo tee /sys/class/pwm/pwmchip3/pwm0/duty_cycle
echo 1 | sudo tee /sys/class/pwm/pwmchip3/pwm0/enable
停止
echo 50000 | sudo tee /sys/class/pwm/pwmchip3/pwm0/duty_cycle
echo 1 | sudo tee /sys/class/pwm/pwmchip3/pwm0/enable

source install/setup.bash
sudo -v
ros2 run dock_motor_controller dock_motor_controller
ros2 topic pub --once /dock_motor/command std_msgs/msg/String "{data: run_cw}"
ros2 topic pub --once /dock_motor/command std_msgs/msg/String "{data: run_ccw}"
ros2 topic pub --once /dock_motor/command std_msgs/msg/String "{data: stop}"
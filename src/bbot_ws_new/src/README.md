## 快速开始

```bash

需要打开三个终端

colcon build
或者
colcon build --packages-select bbot_balance_controller

source install/setup.bash

# 1. 启动遥控器节点
source ~/bbot_ws_new/install/setup.bash

ros2 run bbot_rc_receiver rc_node

# 2. 启动 Gazebo 仿真
ros2 launch bbot_bringup bbot_gazebo.launch.py

# 3. 运行控制器
键盘控制
ros2 run bbot_balance_controller balance_controller_keyboard

遥控控制
ros2 run bbot_balance_controller lqr_balance_controller_yaokong


python3 plot_data.py
python3 plot_height_torque.py

```

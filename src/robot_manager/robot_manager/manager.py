import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess


class RobotManager(Node):

    # ===============================
    # 遥控器配置
    # ===============================

    # buttons[0] 控制主控启停
    START_BUTTON_INDEX = 0

    # axes[5] 控制档位
    GEAR_AXIS_INDEX = 5

    # 空档范围
    # -0.1 ~ +0.1认为空档
    GEAR_NEUTRAL_THRESHOLD = 0.1


    def __init__(self):

        super().__init__('robot_manager')


        # 当前主控状态
        self.balance_running = False


        # 订阅遥控器
        self.subscription = self.create_subscription(
            Joy,
            '/rc_input',
            self.rc_callback,
            10
        )


        self.get_logger().info(
            "Robot manager started"
        )

        self.get_logger().info(
            "Start button: buttons[0]"
        )

        self.get_logger().info(
            "Gear check: axes[5], neutral range +-0.1"
        )



    # ===================================
    # 判断是否空档
    # ===================================

    def is_neutral(self, gear):

        if abs(gear) < self.GEAR_NEUTRAL_THRESHOLD:

            return True

        else:

            return False



    # ===================================
    # 启动主控
    # ===================================

    def start_balance(self):

        if self.balance_running:

            return


        self.get_logger().info(
            "Starting balance controller..."
        )


        try:

            result = subprocess.run(
                [
                    "sudo",
                    "systemctl",
                    "start",
                    "balance.service"
                ],
                capture_output=True,
                text=True
            )


            if result.returncode == 0:


                self.balance_running = True


                self.get_logger().info(
                    "Balance controller started"
                )


            else:

                self.get_logger().error(
                    result.stderr
                )


        except Exception as e:

            self.get_logger().error(
                f"Start balance failed: {e}"
            )



    # ===================================
    # 停止主控
    # ===================================

    def stop_balance(self):

        if not self.balance_running:

            return


        self.get_logger().info(
            "Stopping balance controller..."
        )


        try:

            result = subprocess.run(
                [
                    "sudo",
                    "systemctl",
                    "stop",
                    "balance.service"
                ],
                capture_output=True,
                text=True
            )


            if result.returncode == 0:


                self.balance_running = False


                self.get_logger().info(
                    "Balance controller stopped"
                )


            else:

                self.get_logger().error(
                    result.stderr
                )


        except Exception as e:

            self.get_logger().error(
                f"Stop balance failed: {e}"
            )

    def rc_callback(self,msg):
        if len(msg.buttons) <= self.START_BUTTON_INDEX:
            return
        if len(msg.axes) <= self.GEAR_AXIS_INDEX:

            return

        start_button = msg.buttons[
            self.START_BUTTON_INDEX
        ]
        gear_value = msg.axes[
            self.GEAR_AXIS_INDEX
        ]
        neutral = self.is_neutral(
            gear_value
        )
        if start_button == 1:
            if neutral:
                self.start_balance()
            else:
                self.get_logger().warn(
                    f"Start blocked! "
                    f"Gear not neutral: axes[5]={gear_value:.3f}"
                )
        else:
            self.stop_balance()
def main():
    rclpy.init()
    node = RobotManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()

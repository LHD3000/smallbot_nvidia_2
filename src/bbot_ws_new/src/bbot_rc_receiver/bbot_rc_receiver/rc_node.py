import rclpy
import time
import os
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trackmsg.msg import BalanceDate
from bbot_rc_receiver.crsf_demo.receiver import CrsfReceiver
from bbot_rc_receiver.crsf_demo.serial_io import (
    SerialConfig,
    SerialPort
)
from bbot_rc_receiver.crsf_demo.telemetry import (
    TelemetrySource
)


class CRSFNode(Node):
    # 多少秒没成功收到数据就重开串口
    STALL_TIMEOUT_SEC = 2.0

    def __init__(self):
        super().__init__('crsf_receiver')
        self.last_error_time = 0
        self.rc_connected = False
        self.last_good_time = time.time()

        self.publisher = self.create_publisher(
            Joy,
            '/rc_input_b1',
            10
        )
        self.balance_subscription = self.create_subscription(
            BalanceDate,
            '/balance_date_b1',
            self.balance_callback,
            10
        )

        self._open_serial_blocking()

        self.timer = self.create_timer(
            0.001,
            self.timer_callback
        )

    # ============================================================
    # 阻塞式打开串口 (等设备出现)
    # ============================================================
    def _open_serial_blocking(self):
        while not os.path.exists("/dev/rc_receiver"):
            self.get_logger().warn("Waiting /dev/rc_receiver ...")
            time.sleep(1)

        serial_cfg = SerialConfig(
            port="/dev/rc_receiver",
            baudrate=420000
        )
        self.port = SerialPort(serial_cfg)
        self.telemetry = TelemetrySource(interval=0.1)
        self.receiver = CrsfReceiver(
            port=self.port,
            telemetry=self.telemetry,
            on_channels=self.on_channels
        )
        try:
            if not self.port.is_open:
                self.port.open()
            self.get_logger().info(
                "CRSF Receiver started /dev/rc_receiver @420000"
            )
            self.last_good_time = time.time()
        except Exception as e:
            self.get_logger().error(f"CRSF open failed:{e}")

    # ============================================================
    # 串口 tick + 超时看门狗
    # ============================================================
    def timer_callback(self):
        try:
            self.receiver._tick()
        except Exception as e:
            now = time.time()
            if now - self.last_error_time > 2:
                self.get_logger().warn(f"CRSF communication error:{e}")
                self.last_error_time = now

        # 看门狗: 超过 STALL_TIMEOUT_SEC 没成功发过 Joy 就重开
        idle = time.time() - self.last_good_time
        if idle > self.STALL_TIMEOUT_SEC:
            self.get_logger().error(
                f"No RC data for {idle:.1f}s, reopening serial..."
            )
            try:
                self.port.close()
            except Exception:
                pass
            self._open_serial_blocking()

    def balance_callback(self, msg):
        try:
            self.telemetry.set_target_speed(float(msg.target_speed))
            self.telemetry.set_battery_soc(float(msg.battery_soc))
            self.telemetry.set_battery_temperature(float(msg.battery_temperature))
        except Exception as e:
            self.get_logger().error(f"Telemetry error:{e}")

    # 三档开关通道
    THREE_POS_SWITCH_CH = 6

    def normalize(self, ch):
        v = (float(ch) - 992.0) / 820.0
        if v > 1.0:  v = 1.0
        if v < -1.0: v = -1.0
        return float(v)

    def normalize_3pos(self, ch):
        ch = float(ch)
        if ch < 600.0:  return -1.0
        elif ch > 1400.0: return 1.0
        else:            return 0.0

    def on_channels(self, channels):
        if len(channels) < 16:
            return
        try:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [
                self.normalize(channels[0]),
                self.normalize(channels[1]),
                self.normalize(channels[2]),
                self.normalize(channels[3]),
                self.normalize(channels[4]),
                self.normalize(channels[5]),
                self.normalize(channels[6]),
                self.normalize_3pos(channels[self.THREE_POS_SWITCH_CH])
            ]
            msg.buttons = [
                int(c > 1500)
                for c in channels[4:16]
            ]
            self.publisher.publish(msg)
            self.last_good_time = time.time()
        except Exception as e:
            self.get_logger().warn(f"on_channels publish failed: {e}")

    def destroy_node(self):
        try:
            self.port.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CRSFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

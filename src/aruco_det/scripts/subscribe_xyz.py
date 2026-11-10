#!/usr/bin/python3
"""Print fresh marker-centre coordinates in metres (X right, Y down, Z forward)."""
import math
import signal
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


class XYZSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_xyz_subscriber')
        self.topic = self.declare_parameter('topic', '/aruco/pose').value
        self.timeout = self.declare_parameter('timeout', 1.0).value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        self.last_received = None
        self.last_printed = 0.0
        self.subscription = self.create_subscription(
            PoseStamped, self.topic, self.callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.watchdog, clock=rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def callback(self, message):
        age = (self.get_clock().now() - Time.from_msg(message.header.stamp)).nanoseconds / 1e9
        if abs(age) > self.timeout:
            self.get_logger().warning('Pose timestamp outside timeout', throttle_duration_sec=2.0)
            return
        p = message.pose.position
        if not all(math.isfinite(value) for value in (p.x, p.y, p.z)):
            return
        self.last_received = time.monotonic()
        if self.last_received - self.last_printed >= 0.2:
            self.last_printed = self.last_received
            self.get_logger().info(
                f'frame={message.header.frame_id} X={p.x:+.3f} m Y={p.y:+.3f} m Z={p.z:.3f} m')

    def watchdog(self):
        if self.last_received is None or time.monotonic() - self.last_received > self.timeout:
            self.get_logger().warning('No fresh pose on ' + self.topic, throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = XYZSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

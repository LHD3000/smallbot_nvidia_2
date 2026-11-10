#!/usr/bin/python3
"""Display the annotated image with ROS 2 sensor QoS."""
import cv2
import signal
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image


class ImageViewer(Node):
    def __init__(self):
        super().__init__('aruco_view')
        topic = self.declare_parameter('topic', '/aruco_det_image').value
        self.bridge = CvBridge()
        self.frame = None
        self.shown = False
        self.subscription = self.create_subscription(Image, topic, self.receive, 5)
        self.timer = self.create_timer(0.03, self.display)

    def receive(self, message):
        self.frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')

    def display(self):
        if self.frame is not None:
            cv2.imshow('ArUco - XYZ in metres', self.frame)
            cv2.waitKey(1)
            if not self.shown:
                self.get_logger().info('Displaying annotated image')
                self.shown = True


def main():
    rclpy.init()
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

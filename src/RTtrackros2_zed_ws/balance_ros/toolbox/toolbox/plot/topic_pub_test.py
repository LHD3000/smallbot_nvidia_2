import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math


class PubTest(Node):
    def __init__(self):
        super().__init__("node_test")
        self.val1pub = self.create_publisher(Float64, "val1", 10)
        self.val2pub = self.create_publisher(Float64, "val2", 10)
        self.val3pub = self.create_publisher(Float64, "val3", 10)
        self.t = self.create_timer(0.1, self.pub_test)
        self.i = 0

    def pub_test(self):
        msg = Float64()
        msg.data = math.sin(self.i / 10.0)
        self.val1pub.publish(msg)
        msg.data = math.cos(self.i / 10.0)
        self.val2pub.publish(msg)
        msg.data = math.tan(self.i / 10.0)
        self.val3pub.publish(msg)
        self.i += 1


def main():
    rclpy.init()
    node = PubTest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

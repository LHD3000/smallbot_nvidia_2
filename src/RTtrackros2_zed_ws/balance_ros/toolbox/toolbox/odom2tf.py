import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class Odom2Tf(Node):
    def __init__(self):
        super().__init__("odom2tf")
        self.declare_parameter("tf_name", "tf_odom")
        self.tf_name: str = self.get_parameter("tf_name").value
        self.declare_parameter("use_ns_as_tf_prefix", True)
        self.use_ns: bool = self.get_parameter("use_ns_as_tf_prefix").value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = (
            (self.get_namespace() + "/" + self.tf_name) if self.use_ns else self.tf_name
        )
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Odom2Tf()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Turns the motor board's best-effort wheel_odom into a reliable /odom plus the odom TF.

The micro-controller publishes best effort (reliable would block it on every message),
but nav2 and friends subscribe to /odom with reliable QoS and expect odom -> base_footprint on /tf.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomTf(Node):

    def __init__(self):
        super().__init__('odom_tf')
        # Set to false once something else (e.g. robot_localization) owns odom -> base_footprint.
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Odometry, 'wheel_odom', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)
        if not self.publish_tf:
            return
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

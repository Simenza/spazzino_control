#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import math
import transforms3d as t3d
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Opens serial to Arduino
        self.ser = serial.Serial('/dev/arudino', 115200, timeout=0.1)

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # cmd_vel subscriber
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # serial reading timer
        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def cmd_vel_callback(self, msg: Twist):
        """
        Sends vel commands to Arduino.
        Protocol form: "V {v_lin} {v_ang}\n"
        """
        command = f"V {msg.linear.x:.3f} {msg.angular.z:.3f}\n"
        self.ser.write(command.encode())
        self.get_logger().info(f"Comando inviato: V {msg.linear.x:.3f} {msg.angular.z:.3f}\n")

    def read_serial(self):
        """
        Reads odometry from Arduino.
        Protocol form: "O x y th\n"
        """
        line = self.ser.readline().decode().strip()
        if not line:
            return

        try:
            if line.startswith("O"):
                _, x, y, th = line.split()
                self.x = float(x)
                self.y = float(y)
                self.th = float(th)
                self.publish_odom()
        except Exception as e:
            self.get_logger().warn(f"Serial parsing error: {e}")

    def publish_odom(self):
        # Odometry msg
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = t3d.euler.euler2quat(0, 0, self.th)  # roll=0, pitch=0, yaw=th
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]

        self.odom_pub.publish(odom)

        # TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

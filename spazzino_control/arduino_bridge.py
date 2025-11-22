#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import transforms3d as t3d
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to /cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Timer for serial read
        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def cmd_vel_callback(self, msg: Twist):
        """
        Sends vel commands to Arduino.
        Protocol: "V {v_lin} {v_ang}\n"
        """
        command = f"V {msg.linear.x:.3f} {msg.angular.z:.3f}\n"
        self.ser.write(command.encode())
        self.get_logger().debug(f"Comando inviato: {command.strip()}")

    def read_serial(self):
        """
        Reads data from Arduino:
        - "O x y th" -> odometry
        - "J pos_left pos_right" -> joint states (radians)
        """
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line:
            return

        try:
            if line.startswith("O"):
                _, x, y, th = line.split()
                self.x = float(x)
                self.y = float(y)
                self.th = float(th)
                self.publish_odom()

            elif line.startswith("J"):
                _, pos_left, pos_right = line.split()
                self.publish_joint_states(float(pos_left), float(pos_right))

        except Exception as e:
            self.get_logger().warn(f"Serial parsing error: {e}, line: {line}")

    def publish_odom(self):
        # Odometry msg
        odom = Odometry()
        now = self.get_clock().now().to_msg()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "body_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = t3d.euler.euler2quat(0, 0, self.th)
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]

        self.odom_pub.publish(odom)

        # TF odom -> body_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "body_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self, pos_left, pos_right):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [pos_left, pos_right]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


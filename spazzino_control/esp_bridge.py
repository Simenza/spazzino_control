#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from transforms3d.euler import euler2quat 

class ESPBridge(Node):
    def __init__(self):
        super().__init__('esp_bridge')

        # Parameters 
        self.declare_parameter('esp_ip', '192.168.4.1')
        self.declare_parameter('esp_port', 8888)

        esp_ip = self.get_parameter('esp_ip').value
        esp_port = self.get_parameter('esp_port').value

        # TCP connection to ESP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((esp_ip, esp_port))
        self.get_logger().info(f"Connesso all’ESP su {esp_ip}:{esp_port}")

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # cmd_vel subscriber
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ESP reading thread
        self.running = True
        self.reader_thread = threading.Thread(target=self.read_from_esp, daemon=True)
        self.reader_thread.start()

        # Odometry parameters
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cmd_vel_callback(self, msg: Twist):
        cmd = f"{msg.linear.x:.3f},{msg.angular.z:.3f}\n"
        try:
            self.sock.send(cmd.encode())
        except Exception as e:
            self.get_logger().error(f"Sending error: {e}")

    def read_from_esp(self):
        buffer = ""
        while self.running:
            try:
                data = self.sock.recv(1024).decode()
                if not data:
                    continue
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.process_encoder_data(line.strip())
            except Exception as e:
                self.get_logger().error(f"Reading error: {e}")
                self.running = False

    def process_encoder_data(self, line: str):
        try:
            dl_str, dr_str = line.split(',')
            dl = int(dl_str)
            dr = int(dr_str)

            # Parameters (to calibrate)
            TICKS_PER_M = 1000.0
            WHEEL_BASE = 0.20

            # Tracking movements
            d_left = dl / TICKS_PER_M
            d_right = dr / TICKS_PER_M
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / WHEEL_BASE

            self.x += d_center * math.cos(self.theta + d_theta/2)
            self.y += d_center * math.sin(self.theta + d_theta/2)
            self.theta += d_theta

            # Publishing odometry 
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y

            q = euler2quat(0, 0, self.theta)  # [w, x, y, z]
            odom.pose.pose.orientation.x = q[1]  # ROS [x, y, z, w]
            odom.pose.pose.orientation.y = q[2]
            odom.pose.pose.orientation.z = q[3]
            odom.pose.pose.orientation.w = q[0]

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().warn(f"Not valid encoder data: {line} ({e})")

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

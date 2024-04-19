#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist

class mcqueen_controller(Node):

    def __init__(self):
        super().__init__("mcqueen_controller")
        try:
            self.ser = serial.Serial('/dev/ttyACM0',9600)
            self.ser.reset_input_buffer()
        except:
            self.get_logger().info("Serial not found")
        self.mcqueen_motor_subscription = self.create_subscription(Twist, "/mcqueen_cmdVel", self.motor_callback, 10)
        self.get_logger().info(f"mcqueen controller started")

    def motor_callback(self, msg: Twist):
        self.ser.write(f"{msg.linear.x}, {msg.linear.y}, {msg.linear.z}, {msg.angular.x}, {msg.angular.y}, {msg.angular.z}\n".encode('utf-8'))
        line = self.ser.readline().decode('utf-8').rstrip()
        self.get_logger().info(f"Received {line}")


def main(args=None):
    rclpy.init(args=args)
    node=mcqueen_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
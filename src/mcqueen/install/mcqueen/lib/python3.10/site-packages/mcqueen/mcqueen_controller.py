#!/usr/bin/env python3
import rclpy
import time
import threading
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.srv import LoadMap

class mcqueen_controller(Node):

    def __init__(self):
        super().__init__("mcqueen_controller")
        self.mcqueen_motor_subscription = self.create_subscription(Twist, "/mcqueen_cmdVel", self.motor_callback, 10)
        self.get_logger().info(f"mcqueen controller started")

    def motor_callback(self, msg: Twist):
        self.get_logger().info(f"{msg.linear.x}, {msg.linear.y}, {msg.linear.z}, {msg.angular.x}, {msg.angular.y}, {msg.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node=mcqueen_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
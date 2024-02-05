#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import turtle
import numpy as np

class turtle_bot_interface(Node):

    def __init__(self):
        super().__init__("turtle_bot_interface")
        self.save=(input("Save route? (y/n): ")=='y')
        if self.save:
            self.file_name=input("file name (without .txt): ")
            self.file_path="src/turtle_bot/resource/"+self.file_name+".txt"
        self.x = 0
        self.y = 0
        self.s = turtle.getscreen()
        self.t = turtle.Turtle()
        self.get_logger().info("turtle bot interface started")
        self.turtle_bot_suscriber = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 10)
        

    def position_callback(self, msg: Twist):
        self.x = msg.linear.x
        self.y = msg.linear.y
        self.t.goto(int(100*self.x),int(100*self.y))
        self.get_logger().info(f'moving to ({msg.linear.x}, {msg.linear.y})')
        if self.save:
            with open(self.file_path, 'a') as file:
                file.write(f'{self.x},{self.y}\n')
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node=turtle_bot_interface()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
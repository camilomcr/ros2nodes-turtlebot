#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
    

class turtle_bot_interface(Node):

    def __init__(self):
        super().__init__("turtle_bot_interface")
        self.x = []
        self.y = []
        plt.ion()
        self.get_logger().info("turtle bot interface started")
        self.turtle_bot_suscriber = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 10)
        

    def position_callback(self, msg: Twist):
        self.x.append(msg.linear.x)
        self.y.append(msg.linear.y)
        plt.plot(self.x,self.y, linestyle='-')
        self.get_logger().info(f'moving to ({msg.linear.x}, {msg.linear.y})')
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node=turtle_bot_interface()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
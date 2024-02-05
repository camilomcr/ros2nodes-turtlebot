#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class turtle_bot_player(Node):

    def __init__(self):
        super().__init__("turtle_bot_player")
        self.cmdVel_publisher = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
        self.get_logger().info("turtle bot player started")
        self.timer1 = self.create_timer(0.3, self.set_cmdVel)

    def set_cmdVel(self):
        msg=Twist()

        self.cmdVel_publisher.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node=turtle_bot_player()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
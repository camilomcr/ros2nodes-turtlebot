#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import turtle

class turtle_bot_interface(Node):

    def __init__(self):
        super().__init__("turtle_bot_interface")
        self.save=(input("Save route? (y/n): ")=='y')
        if self.save:
            self.Lx = 0
            self.Ly = 0
            self.Lz = 0
            self.Ax = 0
            self.Ay = 0
            self.Az = 0
            self.file_name=input("file name (without .txt): ")
            self.file_path="src/turtle_bot/resource/"+self.file_name+".txt"
            with open(self.file_path, 'a') as file:
                file.write(f'Lx,Ly,Lz,Ax,Ay,Az\n')
            self.turtle_bot_vel_suscriber = self.create_subscription(Twist, "/turtlebot_cmdVel", self.save_callback, 10)
        
        self.posX = 0
        self.posY = 0
        self.s = turtle.getscreen()
        self.t = turtle.Turtle()
        self.get_logger().info("turtle bot interface started")
        self.turtle_bot_pos_suscriber = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 10)
            
        

    def position_callback(self, msg: Twist):
        self.posX = msg.linear.x
        self.posY = msg.linear.y
        self.t.goto(int(100*self.posX),int(100*self.posY))
        self.get_logger().info(f'moving to ({self.posX}, {self.posX})')
        

    def save_callback(self, msg: Twist):
        self.Lx = msg.linear.x
        self.Ly = msg.linear.y
        self.Lz = msg.linear.z
        self.Ax = msg.angular.x
        self.Ay = msg.angular.y
        self.Az = msg.angular.z

        with open(self.file_path, 'a') as file:
            file.write(f'{self.Lx},{self.Ly},{self.Lz},{self.Ax},{self.Ay},{self.Az}\n')

def main(args=None):
    rclpy.init(args=args)
    node=turtle_bot_interface()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
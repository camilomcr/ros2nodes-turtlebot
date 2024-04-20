#!/usr/bin/env python3
import rclpy
import serial
import math
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
        self.rpm=160
        self.rWheel=0.03
        self.t=0.3/60
        self.r=0
        self.theta=90
        self.mcqueen_pos_publisher = self.create_publisher(Twist, "/mcqueen_position", 10)
        self.mcqueen_motor_subscription = self.create_subscription(Twist, "/mcqueen_cmdVel", self.motor_callback, 10)
        self.get_logger().info(f"mcqueen controller started")

    def motor_callback(self, msg: Twist):
        self.ser.write(f"{msg.linear.x}, {msg.linear.y}, {msg.linear.z}, {msg.angular.x}, {msg.angular.y}, {msg.angular.z}\n".encode('utf-8'))
        line = self.ser.readline().decode('utf-8').rstrip()
        self.get_logger().info(f"Received {line}")
        
        lineParts = line.split(" ")
        if line!="Stopped":
            if lineParts[0]=="Forward" or lineParts[0]=="backwards":
                self.r=self.r+(int(lineParts[1])/255)*self.rpm*self.t*2*math.pi*self.rWheel
            if lineParts[0]=="Clockwise" or lineParts[0]=="Counterclockwise":
                self.theta=self.theta+math.degrees((int(lineParts[1])/255)*self.rpm*self.t*2*math.pi)
        posX=self.r*math.cos(math.radians(self.theta))
        posY=self.r*math.sin(math.radians(self.theta))
        msg=Twist()
        msg.linear.x=posX
        msg.linear.y=posY
        self.mcqueen_pos_publisher.publish(msg)

        


def main(args=None):
    rclpy.init(args=args)
    node=mcqueen_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
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
            self.ser = serial.Serial('/dev/ttyACM0',57600)
            self.ser.reset_input_buffer()
        except:
            self.get_logger().info("Serial not found")
        self.rWheel=0.0325
        self.lWheel=0.127
        self.t=0.001
        self.Npulsos=156.25
        self.lastThetaR=0
        self.lastThetaL=0
        self.lastX=0
        self.lastY=0
        self.lastPhi=0
        self.mcqueen_pos_publisher = self.create_publisher(Twist, "/mcqueen_position", 10)
        self.mcqueen_motor_subscription = self.create_subscription(Twist, "/mcqueen_cmdVel", self.motor_callback, 10)
        self.get_logger().info(f"mcqueen controller started")

    def motor_callback(self, msg: Twist):
        self.ser.write(f"{msg.linear.x}, {msg.linear.y}, {msg.linear.z}, {msg.angular.x}, {msg.angular.y}, {msg.angular.z}\n".encode('utf-8'))
        line = self.ser.readline().decode('utf-8').rstrip()
        self.get_logger().info(f"Received {line}")
        msg=Twist()
        
        lineParts = line.split(",")
        if lineParts[0]!="Stopped":
            MOTR_TICKS=float(lineParts[2])
            MOTL_TICKS=float(lineParts[3])
        else:
            MOTR_TICKS=float(lineParts[1])
            MOTL_TICKS=float(lineParts[2])

        thetaR=(MOTR_TICKS*math.pi*2)/self.Npulsos
        thetaL=(MOTL_TICKS*math.pi*2)/self.Npulsos
        dthetaR=(thetaR-self.lastThetaR)/self.t
        dthetaL=(thetaL-self.lastThetaL)/self.t
        phi=self.lastPhi+self.rWheel*self.t*((dthetaR-dthetaL)/self.lWheel)
        x=self.lastX+math.cos(phi)*self.t*((dthetaR+dthetaL)/2)
        y=self.lastY+math.sin(phi)*self.t*((dthetaR+dthetaL)/2)

        self.lastThetaR=thetaR
        self.lastThetaL=thetaL
        self.lastPhi=phi
        self.lastX=x
        self.lastY=y

        msg.linear.x = x
        msg.linear.y = y
        self.mcqueen_pos_publisher.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    node=mcqueen_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
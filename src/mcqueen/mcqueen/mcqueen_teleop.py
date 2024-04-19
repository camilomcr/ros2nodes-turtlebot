#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses, time, argparse

key=''
pressed = False
lastTime = time.time()
dif = 0.2
parser = argparse.ArgumentParser()
parser.add_argument('--linear', '-l', type=float, default=5.0,help='linear velocity')
parser.add_argument('--angular', '-a', type=float, default=5.0, help='angular velocity')
args = parser.parse_args()

def get_key(stdscr):
    global key
    global pressed
    global lastTime
    stdscr.nodelay(True)
    stdscr.timeout(0)
    try:
        key=stdscr.getkey()
        lastTime = time.time()
        pressed = True
    except curses.error:
        if (time.time()-lastTime) > dif:
            pressed = False
   

class mcqueen_teleop(Node):

    def __init__(self):
        super().__init__("mcqueen_teleop")
        self.cmdVel_publisher = self.create_publisher(Twist, "/mcqueen_cmdVel", 10)
        self.get_logger().info("mcqueen teleop started")
        self.get_logger().info("Use A for left, D for right, W for up and S for down")
        self.timer1 = self.create_timer(0.3, self.set_cmdVel)
        self.timer2 = self.create_timer(0.01, self.wrapper)

    def wrapper(self):
        curses.wrapper(get_key)

    def set_cmdVel(self):
        msg=Twist()
        if not pressed:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            if key == 'd':
                msg.angular.z = args.angular
            elif key == 'a':
                msg.angular.z = -1*args.angular
            elif key == 'w':
                msg.linear.x = args.linear
            elif key == 's':
                msg.linear.x = -1*args.linear
        self.cmdVel_publisher.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node=mcqueen_teleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
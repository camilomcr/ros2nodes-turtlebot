#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.srv import LoadMap

class turtle_bot_player(Node):

    def __init__(self):
        super().__init__("turtle_bot_player")
        self.map_path = ''
        self.cmdVel_publisher = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
        self.server = self.create_service(LoadMap, "/player_service", self.follow_path )
        self.get_logger().info(f"Turtle bot player started")

    def follow_path(self, request: LoadMap.Request, response: LoadMap.Response):
        self.map_path = "src/turtle_bot/resource/"+request.map_url+".txt"
        self.get_logger().info(f"Service called {self.map_path}")
        try:
            threading.Thread(target=self.set_cmdVel).start()
            response.result=0
        except Exception as e:
            response.result=1
            self.get_logger().info(f"Error with the file")
        return response
    
    def set_cmdVel(self):
        self.get_logger().info(f"Setting velocity")
        msg=Twist()
        with open(self.map_path, 'r') as file:
                self.get_logger().info(f"File was opened")
                next(file)
                for line in file:
                    line=line.strip().split(',')
                    self.get_logger().info(f"{line}")
                    msg.linear.x = float(line[0])
                    msg.linear.y = float(line[1])
                    msg.linear.z = float(line[2])
                    msg.angular.x = float(line[3])
                    msg.angular.y = float(line[4])
                    msg.angular.z = float(line[5])
                    self.cmdVel_publisher.publish(msg)
                    time.sleep(0.3)
   

def main(args=None):
    rclpy.init(args=args)
    node=turtle_bot_player()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
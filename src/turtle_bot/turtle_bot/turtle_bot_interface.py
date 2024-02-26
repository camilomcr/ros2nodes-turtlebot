#!/usr/bin/env python3
import rclpy
import threading
import tkinter as tk
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.srv import LoadMap
from matplotlib import pyplot as plt



class turtle_bot_interface(Node):

    def __init__(self):
        super().__init__("turtle_bot_interface")
        self.get_logger().info(f'turtle bot interface started')             
       
    def position_loop(self):
        self.posX = 0
        self.posY = 0
        plt.ion()
        self.turtle_bot_pos_suscriber = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 10)
        self.get_logger().info("turtle bot position drawer started")
        

    def save_loop(self, file_name: str):
        self.Lx = 0
        self.Lz = 0
        self.Ly = 0
        self.Ax = 0
        self.Ay = 0
        self.Az = 0
        self.file_name= file_name
        self.file_path="src/turtle_bot/resource/"+self.file_name+".txt"
        with open(self.file_path, 'a') as file:
            file.write(f'Lx,Ly,Lz,Ax,Ay,Az\n')
        self.turtle_bot_vel_suscriber = self.create_subscription(Twist, "/turtlebot_cmdVel", self.save_callback, 10)
        self.get_logger().info("turtle bot saver started")

    def player_loop(self, file_name: str):
        self.client = self.create_client(LoadMap, "/player_service")
        self.req = LoadMap.Request()
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service unavailable, waiting...')
        self.follow_path(file_name)
        self.get_logger().info("turtle bot path follower started")

    def position_callback(self, msg: Twist):
        
        self.posX = msg.linear.x
        self.posY = msg.linear.y
        plt.scatter(int(100*self.posX),int(100*self.posY),c='r')
        plt.xlim(-300,300)
        plt.ylim(-300,300)
        plt.pause(0.01)
        self.get_logger().info(f'[drawer] moving to ({self.posX}, {self.posX})')
        

    def save_callback(self, msg: Twist):
        self.Lx = msg.linear.x
        self.Ly = msg.linear.y
        self.Lz = msg.linear.z
        self.Ax = msg.angular.x
        self.Ay = msg.angular.y
        self.Az = msg.angular.z

        with open(self.file_path, 'a') as file:
            file.write(f'{self.Lx},{self.Ly},{self.Lz},{self.Ax},{self.Ay},{self.Az}\n')

    def follow_path(self, file_name: str):
        self.req.map_url = file_name
        self.future = self.client.call(self.req)
        return self.future.result()

class gui_class():

    def __init__(self, args):
        self.gui = tk.Tk()
        self.gui.geometry("400x250")
        self.gui.title('Turtle bot interface')
        self.button_position = tk.Button(self.gui, text='Position tracker', width=25, command=self.gui_position)
        self.save_text_box = tk.Text(self.gui,height= 1, width=20)
        self.button_save = tk.Button(self.gui, text='Save position', width=25, command=self.gui_save)
        self.follow_text_box = tk.Text(self.gui,height= 1, width=20)
        self.button_follow = tk.Button(self.gui, text='Follow path', width=25, command=self.gui_follow)
        self.button_position.pack(pady=10)
        self.save_text_box.pack(pady=10)
        self.button_save.pack(pady=10)
        self.follow_text_box.pack(pady=10)
        self.button_follow.pack(pady=10)
        threading.Thread(target=self.start_node, args=(args,)).start()
        self.gui.mainloop()
    
    def start_node(self, args):
        rclpy.init(args=args)
        self.node=turtle_bot_interface()
        rclpy.spin(self.node)
        rclpy.shutdown()
    
    def gui_position(self):
        self.node.position_loop()

    def gui_save(self):
        self.node.save_loop(self.save_text_box.get(1.0, "end-1c"))

    def gui_follow(self):
        self.node.player_loop(self.follow_text_box.get(1.0, "end-1c"))


def main(args=None):
        gui=gui_class(args=args)

if __name__ == '__main__':
    main()
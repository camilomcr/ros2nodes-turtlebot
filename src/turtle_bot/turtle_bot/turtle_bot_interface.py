#!/usr/bin/env python3
import rclpy
import threading
import tkinter as tk
import os
from tkinter import messagebox
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.srv import LoadMap
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg



class turtle_bot_interface(Node):

    def __init__(self):
        super().__init__("turtle_bot_interface")
        self.get_logger().info(f'turtle bot interface started')             
       
    def position_loop(self, gui: tk.Tk, open_pos: bool):
        if not open_pos:
            self.plot_frame.destroy()
            self.destroy_subscription(self.turtle_bot_pos_suscriber)
            return
        self.pos = False
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.plot_frame = tk.Frame(gui)
        self.plot_frame.grid(row=0, column=2, rowspan=5)
        self.subplot= self.fig.add_subplot()
        self.subplot.set_xlim(-250, 250)
        self.subplot.set_ylim(-250, 250)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack()
        self.turtle_bot_pos_suscriber = self.create_subscription(Twist, "/turtlebot_position", self.position_callback, 10)
        self.get_logger().info("turtle bot position drawer started")

    def position_callback(self, msg: Twist):
        if self.pos==False:
            self.posX = int(100*msg.linear.x)
            self.posY = int(100*msg.linear.y)
            self.pos=True
            return
        self.subplot.plot([self.posX, int(100*msg.linear.x)], [self.posY, int(100*msg.linear.y)],c='r')
        self.posX = int(100*msg.linear.x)
        self.posY = int(100*msg.linear.y)
        self.canvas.draw()
        

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
    
    def save_callback(self, msg: Twist):
        self.Lx = msg.linear.x
        self.Ly = msg.linear.y
        self.Lz = msg.linear.z
        self.Ax = msg.angular.x
        self.Ay = msg.angular.y
        self.Az = msg.angular.z

        with open(self.file_path, 'a') as file:
            file.write(f'{self.Lx},{self.Ly},{self.Lz},{self.Ax},{self.Ay},{self.Az}\n')

    def player_loop(self, file_name: str):
        self.client = self.create_client(LoadMap, "/player_service")
        self.req = LoadMap.Request()
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service unavailable, waiting...')
        self.follow_path(file_name)
        self.get_logger().info("turtle bot path follower started")
        

    def follow_path(self, file_name: str):
        self.req.map_url =  file_name
        self.future = self.client.call(self.req)
        return self.future.result()
    
    def stop_save(self):
        self.destroy_subscription(self.turtle_bot_vel_suscriber)

class gui_class():

    def __init__(self, args):
        threading.Thread(target=self.start_node, args=(args,)).start()
        self.gui = tk.Tk()
        self.gui.title('Turtle bot interface')
        self.saving=False
        self.button_position = tk.Button(self.gui, text='Position tracker', width=25, command=self.gui_position)
        self.save_text_box = tk.Text(self.gui,height= 1, width=20)
        self.button_save = tk.Button(self.gui, text='Save position', width=15, command=self.gui_save)
        self.button_stop_save = tk.Button(self.gui, text='Stop saving position', width=15, command=self.gui_stop_save)
        self.follow_text_box = tk.Text(self.gui,height= 1, width=20)
        self.button_follow = tk.Button(self.gui, text='Follow path', width=25, command=self.gui_follow)


        self.button_position.grid(row = 0, column = 0, columnspan=2, pady=10)
        tk.Label(self.gui, text="Save file name").grid(row=1, column=0, pady=10)
        self.save_text_box.grid(row = 1, column = 1,pady=10)
        self.button_save.grid(row = 2, column = 0, pady=10)
        self.button_stop_save.grid(row = 2, column = 1, pady=10)
        tk.Label(self.gui, text="Path file name").grid(row=3, column=0, pady=10)
        self.follow_text_box.grid(row = 3, column = 1, pady=10)
        self.button_follow.grid(row = 4, column = 0, columnspan=2, pady=10)
        self.open_pos = False
        self.gui.mainloop()
    
    def start_node(self, args):
        rclpy.init(args=args)
        self.node=turtle_bot_interface()
        rclpy.spin(self.node)
        rclpy.shutdown()
    
    def gui_position(self):
        self.open_pos= not self.open_pos
        threading.Thread(target=self.node.position_loop, args=(self.gui,self.open_pos,)).start()

    def gui_save(self):
        if self.saving:
            messagebox.showwarning("Warning","A route is being saved")
            return
        text = self.save_text_box.get(1.0, "end-1c")
        if text=='':
            messagebox.showwarning("Warning","Invalid file name")
            return
        self.saving = True
        threading.Thread(target=self.node.save_loop, args=(text,)).start()
    
    def gui_stop_save(self):
        if not self.saving:
            messagebox.showwarning("Warning","There is no route being saved")
            return
        self.saving = False
        threading.Thread(target=self.node.stop_save).start()

    def gui_follow(self):
        text = self.follow_text_box.get(1.0, "end-1c")
        if not os.path.exists("src/turtle_bot/resource/"+text+".txt"):
            messagebox.showwarning("Warning","File not found")
            return
        threading.Thread(target=self.node.player_loop, args=(text,)).start()


def main(args=None):
        gui=gui_class(args=args)

if __name__ == '__main__':
    main()
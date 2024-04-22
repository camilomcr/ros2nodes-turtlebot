#!/usr/bin/env python3
import rclpy
import threading
import tkinter as tk
import os
from tkinter import messagebox
from tkinter.filedialog import asksaveasfilename
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.srv import LoadMap
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg



class mcqueen_interface(Node):

    def __init__(self):
        super().__init__("mcqueen_interface")
        self.get_logger().info(f'mcqueen interface started')             
       
    def position_loop(self, gui: tk.Tk, open_pos: bool):
        if not open_pos:
            file_path = asksaveasfilename(
                defaultextension='.jpg',
                filetypes=[("PNG files", '*.png'), ("JPEG files", '*.jpg'), ("All files", '*.*')],
                title="Save the figure")
            if file_path:
                self.canvas.print_figure(file_path)
                messagebox.showinfo("Saved", "The figure has been saved")
            self.plot_frame.destroy()
            self.destroy_subscription(self.mcqueen_pos_suscriber)
            return
        self.pos = False
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.plot_frame = tk.Frame(gui)
        self.plot_frame.grid(row=0, column=2, rowspan=6)
        self.subplot= self.fig.add_subplot()
        self.subplot.set_xlim(-50, 50)
        self.subplot.set_ylim(-50, 50)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack()
        self.mcqueen_pos_suscriber = self.create_subscription(Twist, "/mcqueen_position", self.position_callback, 10)
        self.get_logger().info("mcqueen position drawer started")

    def position_callback(self, msg: Twist):
        if self.pos==False:
            self.posX = int(msg.linear.x)
            self.posY = int(msg.linear.y)
            self.pos=True
            return
        self.subplot.plot([self.posX, int(msg.linear.x)], [self.posY, int(msg.linear.y)],c='r')
        self.posX = int(msg.linear.x)
        self.posY = int(msg.linear.y)
        self.canvas.draw()
        

    def save_loop(self, file_name: str):
        self.Lx = 0
        self.Lz = 0
        self.Ly = 0
        self.Ax = 0
        self.Ay = 0
        self.Az = 0
        self.file_name= file_name
        self.file_path="src/mcqueen/resource/"+self.file_name+".txt"
        with open(self.file_path, 'a') as file:
            file.write(f'Lx,Ly,Lz,Ax,Ay,Az\n')
        self.mcqueen_vel_suscriber = self.create_subscription(Twist, "/mcqueen_cmdVel", self.save_callback, 10)
        self.get_logger().info("mcqueen saver started")
    
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
        self.get_logger().info("mcqueen path follower started")
        

    def follow_path(self, file_name: str):
        self.req.map_url =  file_name
        self.future = self.client.call(self.req)
    
    def stop_save(self):
        self.destroy_subscription(self.mcqueen_vel_suscriber)

class gui_class():

    def __init__(self, args):
        threading.Thread(target=self.start_node, args=(args,)).start()
        self.gui = tk.Tk()
        self.gui.title('mcqueen interface')

        self.saving=False
        self.button_position = tk.Button(self.gui, text='Start position tracker', width=25, command=self.gui_position)
        self.save_text_box = tk.Text(self.gui,height= 1, width=20)
        self.button_save = tk.Button(self.gui, text='Start recording route', width=15, command=self.gui_save)
        self.button_stop_save = tk.Button(self.gui, text='Stop recording route', width=15, command=self.gui_stop_save)
        self.follow_text_box = tk.Text(self.gui,height= 1, width=20)
        self.button_follow = tk.Button(self.gui, text='Start route follower', width=25, command=self.gui_follow)
        self.button_teleop = tk.Button(self.gui, text='Start teleop node', width=15, command=self.start_teleop)
        self.button_player = tk.Button(self.gui, text='Start player service', width=15, command=self.start_player)


        self.button_position.grid(row = 0, column = 0, columnspan=2, pady=10)
        tk.Label(self.gui, text="Save file name").grid(row=1, column=0, pady=10)
        self.save_text_box.grid(row = 1, column = 1,pady=10)
        self.button_save.grid(row = 2, column = 0, pady=10)
        self.button_stop_save.grid(row = 2, column = 1, pady=10)
        tk.Label(self.gui, text="Path file name").grid(row=3, column=0, pady=10)
        self.follow_text_box.grid(row = 3, column = 1, pady=10)
        self.button_follow.grid(row = 4, column = 0, columnspan=2, pady=10)
        self.button_teleop.grid(row = 5, column = 0, pady=10)
        self.button_player.grid(row = 5, column = 1, pady=10)
        self.open_pos = False
        self.gui.mainloop()
    
    def start_node(self, args):
        rclpy.init(args=args)
        self.node=mcqueen_interface()
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
        messagebox.showinfo("Saved", "The route has been saved")
        threading.Thread(target=self.node.stop_save).start()

    def gui_follow(self):
        text = self.follow_text_box.get(1.0, "end-1c")
        if not os.path.exists("src/mcqueen/resource/"+text+".txt"):
            messagebox.showwarning("Warning","File not found")
            return
        threading.Thread(target=self.node.player_loop, args=(text,)).start()

    
    def start_teleop(self):
        os.system("gnome-terminal -x ros2 run mcqueen mcqueen_teleop")
    
    def start_player(self):
        os.system("gnome-terminal -x ros2 run mcqueen mcqueen_player")


def main(args=None):
        gui=gui_class(args=args)

if __name__ == '__main__':
    main()
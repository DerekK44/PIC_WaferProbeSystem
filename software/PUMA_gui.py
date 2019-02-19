# -*- coding: utf-8 -*-
"""
Created on Fri Oct  5 18:14:00 2018

@author: PUMA
"""

from pic_tester_lib import PICtester
from pyluna import Luna
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

from tkinter import *
from tkinter import messagebox
from tkinter import ttk

class PUMA_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PUMA Controller")
        
        """ Create a main frame widget """
        self.mainframe = ttk.Frame(self.root, padding="3 3 12 12")
        self.mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        
        self.stageframe = ttk.LabelFrame(self.mainframe, text="Stage")
        self.stageframe.grid(column=0, row=0, sticky='n')
        
        ttk.Separator(self.mainframe, orient=VERTICAL).grid(column=1, row=0, sticky='ns')
        
        self.middle_frame = ttk.Frame(self.mainframe)
        self.middle_frame.grid(column=2, row=0, sticky='n')
        
        self.lunaframe = ttk.LabelFrame(self.middle_frame, text="Luna")
        self.lunaframe.grid(column=0, row=0, pady=10, sticky='nw')
        
        ttk.Separator(self.middle_frame, orient=HORIZONTAL).grid(column=0, row=1, sticky='ew')
        
        self.alignment_frame = ttk.LabelFrame(self.middle_frame, text="Stage Alignment (rotation correction)")
        self.alignment_frame.grid(column=0, row=2, pady=10, sticky='nw')
        
        ttk.Separator(self.middle_frame, orient=HORIZONTAL).grid(column=0, row=3, sticky='ew')
        
        self.auto_measure_frame = ttk.LabelFrame(self.middle_frame, text="Auto Measurement Setup")
        self.auto_measure_frame.grid(column=0, row=4, pady=10, sticky='nw')
        
        self.stageON = False
        self.lunaON = False
        
        self.build_init_frame()
        self.build_step_frame()
        self.build_move_arrow_frame()
        self.build_monitor_frame()
        self.build_spiral_frame()
        
        self.build_luna_init_frame()
        self.build_luna_params_frame()
        
        self.build_alignment_frame()
        
        self.build_auto_measure_params_frame()
        
        """ Give all the widgets on the screen and give them some padding """
        for child in self.mainframe.winfo_children(): child.grid_configure(padx=5, pady=10)
        
        self.realtime_display()
        
    def build_init_frame(self):
        self.init_frame = ttk.Frame(self.stageframe)
        self.init_frame.grid(column=0, row=0, pady=10)
        
        self.connect_pic_tester = ttk.Button(self.init_frame, text='Connect to PICtester', command=self.open_pic_tester)
        self.connect_pic_tester.grid(column=0, row=0, stick=(W,E))
        
        ttk.Separator(self.init_frame, orient=VERTICAL).grid(column=1, row=0, padx=15, sticky='ns')
        
        self.disconnect_pic_tester = ttk.Button(self.init_frame, text='Disconnect from PICtester', command=self.close_pic_tester)
        self.disconnect_pic_tester.grid(column=2, row=0, stick=(W,E))
        
        self.unload_button = ttk.Button(self.init_frame, text='Center position', command=self.move_to_center_position)
        self.unload_button.grid(column=2, row=1, stick=(W,E))
        
        ttk.Separator(self.init_frame, orient=VERTICAL).grid(column=1, row=1, padx=15, sticky='ns')
        
        self.load_button = ttk.Button(self.init_frame, text='Load position', command=self.move_to_load_position)
        self.load_button.grid(column=0, row=1, stick=(W,E))
    
    def open_pic_tester(self):
        self.pc = PICtester()
        self.stageON = True
        
    def close_pic_tester(self):
        if self.stageON == False:
            messagebox.showinfo(message='Cannot disconnect, PICtester is already disconnected.')
        else:
            self.pc.close()
            self.stageON = False
            print("Connection closed.")
        
    def move_to_center_position(self):
        if self.stageON == False:
            messagebox.showinfo(message='Please connect to the stages first.')
        else:
            messagebox.showinfo(message='Please make sure the fiber arrays are retracted!\n  Stage will move to unload position once this window is closed.')
            
            #BYPASS ANGLE OFFSET
            ao = self.pc.angle_offset
            self.pc.angle_offset = 0.0
            
            self.pc.move_stage(0.0, 0.0)
            
            #REINSTATE ANGLE OFFSET
            self.pc.angle_offset = ao
    
    def move_to_load_position(self):
        if self.stageON == False:
            messagebox.showinfo(message='Please connect to the stages first.')
        else:
            messagebox.showinfo(message='Please make sure the fiber arrays are retracted!\n  Stage will move to load position once this window is closed.')
            
            #BYPASS ANGLE OFFSET
            ao = self.pc.angle_offset
            self.pc.angle_offset = 0.0
            
            self.pc.move_stage(0.0, 75000.0)
            
            #REINSTATE ANGLE OFFSET
            self.pc.angle_offset = ao
    
    def build_step_frame(self):
        self.step_frame = ttk.Frame(self.stageframe)
        self.step_frame.grid(column=0, row=1, pady=10)
        
        self.step_x = StringVar()
        self.step_y = StringVar()
        self.cur_step_x = StringVar()
        self.cur_step_y = StringVar()
        
        """ Initialize the step-sizes """
        self.step_x.set(50.0)
        self.step_y.set(50.0)
        self.cur_step_x.set(50.0)
        self.cur_step_y.set(50.0)
        
        step_x_entry = ttk.Entry(self.step_frame, width=7, textvariable=self.step_x).grid(column=2, row=2, sticky=(W, E))
        step_y_entry = ttk.Entry(self.step_frame, width=7, textvariable=self.step_y).grid(column=2, row=3, sticky=(W, E))
        
        ttk.Label(self.step_frame, width=10, textvariable=self.cur_step_x).grid(column=4, row=2, sticky=(W, E))
        ttk.Label(self.step_frame, width=10, textvariable=self.cur_step_y).grid(column=4, row=3, sticky=(W, E))
        
        ttk.Label(self.step_frame, text="x-step [um]").grid(column=1, row=2, sticky=E)
        ttk.Label(self.step_frame, text="y-step [um]").grid(column=1, row=3, sticky=E)
        
        ttk.Label(self.step_frame, text="Current x-step [um]: ").grid(column=3, row=2, sticky=E)
        ttk.Label(self.step_frame, text="Current y-step [um]: ").grid(column=3, row=3, sticky=E)
        
        ttk.Button(self.step_frame, text="Set X", command=self.set_x).grid(column=5, row=2, sticky=E)
        ttk.Button(self.step_frame, text="Set Y", command=self.set_y).grid(column=5, row=3, sticky=E)
        
    def set_x(self, *args):
        try:
            self.cur_step_x.set(float(self.step_x.get()))
        except ValueError():
            pass

    def set_y(self, *args):
        try:
            self.cur_step_y.set(float(self.step_y.get()))
        except ValueError():
            pass
        
    def build_move_arrow_frame(self):
        self.move_arrow_frame = ttk.LabelFrame(self.stageframe, text="Stage Controller")
        self.move_arrow_frame.grid(column=0, row=2, pady=10)
        
        self.up_button = ttk.Button(self.move_arrow_frame, command=self.moveUp)
        self.up_button.grid(column=2, row=1)
        
        self.down_button = ttk.Button(self.move_arrow_frame, text='test', command=self.moveDown)
        self.down_button.grid(column=2, row=2)
        
        self.left_button = ttk.Button(self.move_arrow_frame, text='test', command=self.moveLeft)
        self.left_button.grid(column=1, row=2)
        
        self.right_button = ttk.Button(self.move_arrow_frame, text='test', command=self.moveRight)
        self.right_button.grid(column=3, row=2)
        
        self.uparrow = PhotoImage(file='imgs/up-arrow.png')
        self.up_button['image'] = self.uparrow
        
        self.downarrow = PhotoImage(file='imgs/down-arrow.png')
        self.down_button['image'] = self.downarrow
        
        self.leftarrow = PhotoImage(file='imgs/left-arrow.png')
        self.left_button['image'] = self.leftarrow
        
        self.rightarrow = PhotoImage(file='imgs/right-arrow.png')
        self.right_button['image'] = self.rightarrow
        
    def moveUp(self, *args):
        """ Move in +y direction by an amount equal to cur_step_y """
        if self.stageON:
            self.pc.move_stage_relative(0.0, float(self.cur_step_y.get()))
            print("Moving in +y by "+str(float(self.cur_step_y.get())))
            self.update_displays()
        else:
            messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')
        
    def moveDown(self, *args):
        """ Move in -y direction by an amount equal to cur_step_y """
        if self.stageON:
            self.pc.move_stage_relative(0.0, -float(self.cur_step_y.get()))
            print("Moving in -y by "+str(float(self.cur_step_y.get())))
            self.update_displays()
        else:
            messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')
    
    def moveLeft(self, *args):
        """ Move in -x direction by an amount equal to cur_step_x """
        if self.stageON:
            self.pc.move_stage_relative(-float(self.cur_step_x.get()), 0.0)
            print("Moving in -x by "+str(float(self.cur_step_x.get())))
            self.update_displays()
        else:
            messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')
        
    def moveRight(self, *args):
        """ Move in +x direction by an amount equal to cur_step_x """
        if self.stageON:
            self.pc.move_stage_relative(float(self.cur_step_x.get()), 0.0)
            print("Moving in +x by "+str(float(self.cur_step_x.get())))
            self.update_displays()
        else:
            messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')
        
    def build_monitor_frame(self):
        self.monitor_frame = ttk.LabelFrame(self.stageframe, text="Status")
        self.monitor_frame.grid(column=0, row=3, pady=10)
        
        self.cur_x = StringVar()
        self.cur_y = StringVar()
        self.cur_power = StringVar()
        
        self.update_displays()
        
        ttk.Label(self.monitor_frame, width=20, text="x [um]").grid(column=2, row=1, sticky=E)
        ttk.Label(self.monitor_frame, width=20, text="y [um]").grid(column=3, row=1, sticky=E)
        ttk.Label(self.monitor_frame, width=20, text="Current position").grid(column=1, row=2, sticky=W)
        self.cur_x_label = ttk.Label(self.monitor_frame, width=10, textvariable=self.cur_x).grid(column=2, row=2, sticky=(W, E))
        self.cur_y_label = ttk.Label(self.monitor_frame, width=10, textvariable=self.cur_y).grid(column=3, row=2, sticky=(W, E))
        
        ttk.Label(self.monitor_frame, width=20, text="Photodetector").grid(column=1, row=3, sticky=W)
        self.cur_power_label = ttk.Label(self.monitor_frame, width=20, textvariable=self.cur_power).grid(column=2, row=3, sticky=(W, E))
        ttk.Label(self.monitor_frame, width=10, text="Watts").grid(column=3, row=3, sticky=W)
        
    def realtime_display(self):
        self.update_displays()
        self.root.after(500, self.realtime_display)
        
    def update_displays(self):
        if self.stageON == False:
            self.cur_x.set("OFF")
            self.cur_y.set("OFF")
            self.cur_power.set("OFF")
        else:
            self.cur_x.set(self.pc.query_position()[0])
            self.cur_y.set(self.pc.query_position()[1])
            self.cur_power.set(round(self.pc.get_power(), 12))
        
    def build_spiral_frame(self):
        self.spiral_frame = ttk.LabelFrame(self.stageframe, text="Spiral Alignment")
        self.spiral_frame.grid(column=0, row=5, pady=10)
        
        self.spiral_radius = StringVar()
        self.spiral_arm_dist = StringVar()
        self.spiral_pts_per_rev = StringVar()
        self.spiral_data_filename = StringVar()
        self.plot_spiral_boolean = StringVar()
        
        
        self.spiral_radius.set(5.0)
        self.spiral_arm_dist.set(0.3)
        self.spiral_pts_per_rev.set(12)
        self.spiral_data_filename.set("spiral_data.csv")
        self.plot_spiral_boolean.set(1)
        
        ttk.Label(self.spiral_frame, width=20, text="Radius [um]").grid(column=1, row=1, sticky=E)
        ttk.Label(self.spiral_frame, width=20, text="Arm distance [um]").grid(column=1, row=2, sticky=E)
        ttk.Label(self.spiral_frame, width=20, text="Points/rotation").grid(column=1, row=3, sticky=E)
        
        ttk.Entry(self.spiral_frame, width=7, textvariable=self.spiral_radius).grid(column=2, row=1, sticky=(W, E))
        ttk.Entry(self.spiral_frame, width=7, textvariable=self.spiral_arm_dist).grid(column=2, row=2, sticky=(W, E))
        ttk.Entry(self.spiral_frame, width=7, textvariable=self.spiral_pts_per_rev).grid(column=2, row=3, sticky=(W, E))
        
        self.spiral_button = ttk.Button(self.spiral_frame, text='Start spiral search', command=self.spiral_search)
        self.spiral_button.grid(column=2, row=4)
        
        ttk.Label(self.spiral_frame, width=20, text="Save data as...").grid(column=1, row=5, sticky=E)
        ttk.Entry(self.spiral_frame, width=12, textvariable=self.spiral_data_filename).grid(column=2, row=5, sticky=(W, E))
        self.spiral_data_save_button = ttk.Button(self.spiral_frame, text='Save spiral data', command=self.save_spiral_data)
        self.spiral_data_save_button.grid(column=3, row=5)
        
        ttk.Radiobutton(self.spiral_frame, text="Show plot window", variable=self.plot_spiral_boolean, value=1).grid(column=0, row=6, sticky=W)
        ttk.Radiobutton(self.spiral_frame, text="Hide plot window", variable=self.plot_spiral_boolean, value=0).grid(column=1, row=6, sticky=W)
        
    def spiral_search(self):
        if self.stageON:
            if int(self.plot_spiral_boolean.get())==1:
                return self.pc.spiral_search(radius_max=float(self.spiral_radius.get()),
                                              arm_dist=float(self.spiral_arm_dist.get()),
                                              dtheta=2*np.pi/float(self.spiral_pts_per_rev.get()),
                                              show_plot=True)
            elif int(self.plot_spiral_boolean.get())==0:
                return self.pc.spiral_search(radius_max=float(self.spiral_radius.get()),
                                              arm_dist=float(self.spiral_arm_dist.get()),
                                              dtheta=2*np.pi/float(self.spiral_pts_per_rev.get()),
                                              show_plot=False)
        else:
            messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')

    def spiral_search_fine(self):
        if self.stageON:
            fine_radius = 4.0
            fine_arm_dist = 0.5
            fine_dtheta = 2*np.pi/16.0
            if int(self.plot_spiral_boolean.get())==1:
                return self.pc.spiral_search(radius_max=fine_radius,
                                              arm_dist=fine_arm_dist,
                                              dtheta=fine_dtheta,
                                              show_plot=True)
            elif int(self.plot_spiral_boolean.get())==0:
                return self.pc.spiral_search(radius_max=fine_radius,
                                              arm_dist=fine_arm_dist,
                                              dtheta=fine_dtheta,
                                              show_plot=False)
        else:
            messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')
        
        
    def save_spiral_data(self):
        self.pc.save_spiral_data(filename=str(self.spiral_data_filename.get()))

    def build_luna_init_frame(self):
        self.luna_init_frame = ttk.Frame(self.lunaframe)
        self.luna_init_frame.grid(column=0, row=0, pady=10)

        self.connect_luna_button = ttk.Button(self.luna_init_frame, text='Connect to LUNA OVA5000', command=self.open_luna)
        self.connect_luna_button.grid(column=0, row=0, stick=W)
        
        ttk.Separator(self.luna_init_frame, orient=VERTICAL).grid(column=1, row=0, padx=15, sticky='ns')
        
        self.disconnect_pic_tester = ttk.Button(self.luna_init_frame, text='Disconnect from LUNA', command=self.close_luna)
        self.disconnect_pic_tester.grid(column=2, row=0, stick=E)
    
    def open_luna(self):
        self.luna = Luna()
        self.lunaON = True
        
    def close_luna(self):
        self.luna.close()
        self.lunaON = False
        print("LUNA connection closed.")
        
    def build_luna_params_frame(self):
        self.luna_params_frame = ttk.Frame(self.lunaframe)
        self.luna_params_frame.grid(column=0, row=1, pady=10)
        
        self.luna_center_wl = StringVar()
        self.luna_scan_range = StringVar()
        self.cur_luna_center_wl = StringVar()
        self.cur_luna_scan_range = StringVar()
        self.luna_wl = StringVar()
        self.cur_luna_wl = StringVar()
        self.cur_DUT = StringVar()
        
        self.plot_data_boolean = StringVar()
        
        self.luna_scan_data_filename = StringVar()
        
        """ Initialize the luna parameters """
        self.luna_center_wl.set(1550.0)
        self.luna_scan_range.set(20.0)
        self.cur_luna_center_wl.set(1550.0)
        self.cur_luna_scan_range.set(20.0)
        self.luna_wl.set(1550.0)
        self.cur_luna_wl.set(1550.0)
        self.cur_DUT.set(float(3.0))
        self.plot_data_boolean.set(1)
        self.luna_scan_data_filename.set("luna_data.csv")
        
        ttk.Entry(self.luna_params_frame, width=7, textvariable=self.luna_center_wl).grid(column=1, row=2, sticky=(W, E))
        ttk.Entry(self.luna_params_frame, width=7, textvariable=self.luna_scan_range).grid(column=1, row=3, sticky=(W, E))
        
        ttk.Label(self.luna_params_frame, width=10, textvariable=self.cur_luna_center_wl).grid(column=3, row=2, sticky=(W, E))
        ttk.Label(self.luna_params_frame, width=10, textvariable=self.cur_luna_scan_range).grid(column=3, row=3, sticky=(W, E))
        
        ttk.Label(self.luna_params_frame, text="Center wavelength [nm]").grid(column=0, row=2, sticky=E)
        ttk.Label(self.luna_params_frame, text="Scan range [nm]").grid(column=0, row=3, sticky=E)
        
        ttk.Label(self.luna_params_frame, text="Current center wavelength [nm]: ").grid(column=2, row=2, sticky=E)
        ttk.Label(self.luna_params_frame, text="Current scan range [nm]: ").grid(column=2, row=3, sticky=E)
        
        ttk.Button(self.luna_params_frame, text="Set", command=self.set_center_wl).grid(column=4, row=2, sticky=E)
        ttk.Button(self.luna_params_frame, text="Set", command=self.set_scan_range).grid(column=4, row=3, sticky=E)
        
        ttk.Label(self.luna_params_frame, text="LUNA wavelength [nm]").grid(column=0, row=4, sticky=E)
        ttk.Entry(self.luna_params_frame, width=7, textvariable=self.luna_wl).grid(column=1, row=4, sticky=(W, E))
        ttk.Label(self.luna_params_frame, text="Current wavelength [nm]: ").grid(column=2, row=4, sticky=E)
        ttk.Label(self.luna_params_frame, width=10, textvariable=self.cur_luna_wl).grid(column=3, row=4, sticky=(W, E))
        ttk.Button(self.luna_params_frame, text="Set", command=self.set_luna_wl).grid(column=4, row=4, sticky=E)
        
        ttk.Label(self.luna_params_frame, text="Current DUT [m]: ").grid(column=2, row=5, sticky=E)
        ttk.Label(self.luna_params_frame, width=10, textvariable=self.cur_DUT).grid(column=3, row=5, sticky=(W, E))
        ttk.Button(self.luna_params_frame, text="Find DUT", command=self.find_DUT).grid(column=4, row=5, sticky=E)
        
        ttk.Radiobutton(self.luna_params_frame, text="Show plot window", variable=self.plot_data_boolean, value=1).grid(column=0, row=5, sticky=W)
        ttk.Radiobutton(self.luna_params_frame, text="Hide plot window", variable=self.plot_data_boolean, value=0).grid(column=1, row=5, sticky=W)
        
        ttk.Button(self.luna_params_frame, text="Start SCAN", width=20, command=self.luna_scan).grid(column=3, row=6, sticky=W, pady=10, padx=5)
        
        ttk.Label(self.luna_params_frame, width=20, text="Save data as...").grid(column=0, row=6, sticky=E)
        ttk.Entry(self.luna_params_frame, width=12, textvariable=self.luna_scan_data_filename).grid(column=1, row=6, sticky=(W, E))
        self.spiral_data_save_button = ttk.Button(self.luna_params_frame, text='Save scan data', command=self.save_scan_data)
        self.spiral_data_save_button.grid(column=2, row=6)
        
    def set_center_wl(self, *args):
        if self.lunaON:
            try:
                self.cur_luna_center_wl.set(float(self.luna_center_wl.get()))    
                self.luna.configure_scan_parameters(center_wl = float(self.cur_luna_center_wl.get()))            
            except ValueError():
                pass
        else:
            messagebox.showinfo(message='Please check that the LUNA OVA5000 is powered on, connected, and calibrated then press "Connect to LUNA OVA5000".')

    def set_scan_range(self, *args):
        if self.lunaON:
            try:
                self.cur_luna_scan_range.set(float(self.luna_scan_range.get()))
                self.luna.configure_scan_parameters(scan_range = float(self.cur_luna_scan_range.get()))
            except ValueError():
                pass
        else:
            messagebox.showinfo(message='Please check that the LUNA OVA5000 is powered on, connected, and calibrated then press "Connect to LUNA OVA5000".')
            
    def set_luna_wl(self, *args):
        if self.lunaON:
            try:
                self.cur_luna_wl.set(self.luna_wl.get())
                self.luna.set_source_wavelength(float(self.cur_luna_wl.get()))
                self.luna.turnon_light_source()
            except ValueError():
                pass
        else:
            messagebox.showinfo(message='Please check that the LUNA OVA5000 is powered on, connected, and calibrated then press "Connect to LUNA OVA5000".')
            
    def find_DUT(self, *args):
        if self.lunaON:
            try:
                self.luna.find_DUT()
                self.cur_DUT.set(float(self.luna.get_DUT()))
            except ValueError():
                pass
        else:
            messagebox.showinfo(message='Please check that the LUNA OVA5000 is powered on, connected, and calibrated then press "Connect to LUNA OVA5000".')
        
    def luna_scan(self, *args):
        if self.lunaON:
            if int(self.plot_data_boolean.get())==1:
                self.luna.scan(plot=True)
            elif int(self.plot_data_boolean.get())==0:
                self.luna.scan(plot=False)
        else:
            messagebox.showinfo(message='Please check that the LUNA OVA5000 is powered on, connected, and calibrated then press "Connect to LUNA OVA5000" prior to attempting scan.')
    
    def save_scan_data(self):
        self.luna.save_scan_data(filename=str(self.luna_scan_data_filename.get()))
        
    def build_alignment_frame(self):
#        self.alignment_frame = ttk.LabelFrame(self.stageframe, text="Stage Alignment (rotation correction)")
#        self.alignment_frame.grid(column=0, row=4, pady=10)
        
        self.alignment_point1_x = StringVar()
        self.alignment_point1_y = StringVar()
        self.alignment_point2_x = StringVar()
        self.alignment_point2_y = StringVar()
        self.cur_alignment_point1_x = StringVar()
        self.cur_alignment_point1_y = StringVar()
        self.cur_alignment_point2_x = StringVar()
        self.cur_alignment_point2_y = StringVar()
        
        self.set_alignment_point1 = ttk.Button(self.alignment_frame, text='Set Current Position as Point 1', command=self.set_alignment_pt1)
        self.set_alignment_point1.grid(column=0, row=1)
        self.set_alignment_point2 = ttk.Button(self.alignment_frame, text='Set Current Position as Point 2', command=self.set_alignment_pt2)
        self.set_alignment_point2.grid(column=0, row=2)
        
        ttk.Label(self.alignment_frame, width=20, text="x [um]").grid(column=1, row=0, sticky=E)
        ttk.Label(self.alignment_frame, width=20, text="y [um]").grid(column=2, row=0, sticky=E)
        ttk.Label(self.alignment_frame, width=20, text="Mask x [um]").grid(column=3, row=0, sticky=E)
        ttk.Label(self.alignment_frame, width=20, text="Mask y [um]").grid(column=4, row=0, sticky=E)
        
        ttk.Entry(self.alignment_frame, width=20, textvariable=self.alignment_point1_x).grid(column=3, row=1, sticky=(W, E))
        ttk.Entry(self.alignment_frame, width=20, textvariable=self.alignment_point1_y).grid(column=4, row=1, sticky=(W, E))
        ttk.Entry(self.alignment_frame, width=20, textvariable=self.alignment_point2_x).grid(column=3, row=2, sticky=(W, E))
        ttk.Entry(self.alignment_frame, width=20, textvariable=self.alignment_point2_y).grid(column=4, row=2, sticky=(W, E))
        
        self.cur_pt1_x_label = ttk.Label(self.alignment_frame, width=20, textvariable=self.cur_alignment_point1_x).grid(column=1, row=1, sticky=(W, E))
        self.cur_pt1_y_label = ttk.Label(self.alignment_frame, width=20, textvariable=self.cur_alignment_point1_y).grid(column=2, row=1, sticky=(W, E))
        self.cur_pt2_x_label = ttk.Label(self.alignment_frame, width=20, textvariable=self.cur_alignment_point2_x).grid(column=1, row=2, sticky=(W, E))
        self.cur_pt2_y_label = ttk.Label(self.alignment_frame, width=20, textvariable=self.cur_alignment_point2_y).grid(column=2, row=2, sticky=(W, E))
        
        self.rotate_coordinates_button = ttk.Button(self.alignment_frame, text='Rotate Coordinate System', command=self.rotate_coordinate_system)
        self.rotate_coordinates_button.grid(column=0, row=3, sticky=E)
        
        self.reset_coordinates_button = ttk.Button(self.alignment_frame, text='Reset (zero) Coordinate System', command=self.reset_coordinate_system)
        self.reset_coordinates_button.grid(column=1, row=3, sticky=E)
            
    def set_alignment_pt1(self, *args):
        if self.stageON == False:
            messagebox.showinfo(message='Please connect to the stages first.')
        else:
            self.cur_alignment_point1_x.set(self.pc.query_position()[0])
            self.cur_alignment_point1_y.set(self.pc.query_position()[1])
        
    def set_alignment_pt2(self, *args):
        if self.stageON == False:
            messagebox.showinfo(message='Please connect to the stages first.')
        else:
            self.cur_alignment_point2_x.set(self.pc.query_position()[0])
            self.cur_alignment_point2_y.set(self.pc.query_position()[1])
        
        
    def rotate_coordinate_system(self, *args):
        if self.stageON == False:
            messagebox.showinfo(message='Please connect to the stages first.')
        else:
            try:
                pt2_stage = (float(self.cur_alignment_point2_x.get()), float(self.cur_alignment_point2_y.get()))
                pt1_stage = (float(self.cur_alignment_point1_x.get()), float(self.cur_alignment_point1_y.get()))
                
                pt2_mask = (float(self.alignment_point2_x.get()), float(self.alignment_point2_y.get()))
                pt1_mask = (float(self.alignment_point1_x.get()), float(self.alignment_point1_y.get()))
                
                theta_stage = np.arctan((pt2_stage[1]-pt1_stage[1])/(pt2_stage[0]-pt1_stage[0]))
                theta_mask = np.arctan((pt2_mask[1]-pt1_mask[1])/(pt2_mask[0]-pt1_mask[0]))
                
                angle_offset = theta_stage - theta_mask
                
                self.pc.angle_offset = angle_offset
                
                magnitude_stage = np.sqrt((pt2_stage[1]-pt1_stage[1])**2 + (pt2_stage[0]-pt1_stage[0])**2)
                magnitude_mask = np.sqrt((pt2_mask[1]-pt1_mask[1])**2 + (pt2_mask[0]-pt1_mask[0])**2)
                delta_magnitude = magnitude_stage - magnitude_mask
                percent_delta_magnitude = 100.0*delta_magnitude/magnitude_mask
                
                messagebox.showinfo(message='Stage Rotation Alignment Complete.\n Angle offset = '+str(round(180*angle_offset/np.pi,6))+' degrees \n Delta magnitude = '+str(round(delta_magnitude,6))+'um ('+str(round(percent_delta_magnitude,6))+'%)')
                
            except ValueError():
                pass
        
    def reset_coordinate_system(self, *args):
        if self.stageON == False:
            messagebox.showinfo(message='Please connect to the stages first.')
        else:
            self.pc.angle_offset = 0.0
            messagebox.showinfo(message='Coordinate system reset to default value of angle-adjust = 0.0 degrees.')
            
    def build_auto_measure_params_frame(self):
        self.auto_measure_params_frame = ttk.Frame(self.auto_measure_frame)
        self.auto_measure_params_frame.grid(column=0, row=0, pady=10)
        
        self.auto_nx = StringVar()
        self.auto_nx.set(4)
        self.auto_cur_nx = StringVar()
        self.auto_cur_nx.set(4)
        
        self.auto_ny = StringVar()
        self.auto_ny.set(4)
        self.auto_cur_ny = StringVar()
        self.auto_cur_ny.set(4)
        
        self.auto_dx = StringVar()
        self.auto_dx.set(500.0)
        self.auto_cur_dx = StringVar()
        self.auto_cur_dx.set(500.0)
        
        self.auto_dy = StringVar()
        self.auto_dy.set(500.0)
        self.auto_cur_dy = StringVar()
        self.auto_cur_dy.set(500.0)
        
        ttk.Label(self.auto_measure_params_frame, text="Number in x-direction").grid(column=0, row=0, sticky=E)
        ttk.Entry(self.auto_measure_params_frame, width=7, textvariable=self.auto_nx).grid(column=1, row=0, sticky=(W, E))
        ttk.Label(self.auto_measure_params_frame, text="Current number in x-direction: ").grid(column=2, row=0, sticky=E)
        ttk.Label(self.auto_measure_params_frame, width=10, textvariable=self.auto_cur_nx).grid(column=3, row=0, sticky=(W, E))
        ttk.Button(self.auto_measure_params_frame, text="Set", command=self.set_auto_nx).grid(column=4, row=0, sticky=W)
        
        ttk.Label(self.auto_measure_params_frame, text="Number in y-direction").grid(column=0, row=1, sticky=E)
        ttk.Entry(self.auto_measure_params_frame, width=7, textvariable=self.auto_ny).grid(column=1, row=1, sticky=(W, E))
        ttk.Label(self.auto_measure_params_frame, text="Current number in y-direction: ").grid(column=2, row=1, sticky=E)
        ttk.Label(self.auto_measure_params_frame, width=10, textvariable=self.auto_cur_ny).grid(column=3, row=1, sticky=(W, E))
        ttk.Button(self.auto_measure_params_frame, text="Set", command=self.set_auto_ny).grid(column=4, row=1, sticky=W)
        
        ttk.Label(self.auto_measure_params_frame, text="Step size in x-direction [um]").grid(column=0, row=2, sticky=E)
        ttk.Entry(self.auto_measure_params_frame, width=7, textvariable=self.auto_dx).grid(column=1, row=2, sticky=(W, E))
        ttk.Label(self.auto_measure_params_frame, text="Current x-step [um]: ").grid(column=2, row=2, sticky=E)
        ttk.Label(self.auto_measure_params_frame, width=10, textvariable=self.auto_cur_dx).grid(column=3, row=2, sticky=(W, E))
        ttk.Button(self.auto_measure_params_frame, text="Set", command=self.set_auto_dx).grid(column=4, row=2, sticky=W)
        
        ttk.Label(self.auto_measure_params_frame, text="Step size in y-direction [um]").grid(column=0, row=3, sticky=E)
        ttk.Entry(self.auto_measure_params_frame, width=7, textvariable=self.auto_dy).grid(column=1, row=3, sticky=(W, E))
        ttk.Label(self.auto_measure_params_frame, text="Current y-step [um]: ").grid(column=2, row=3, sticky=E)
        ttk.Label(self.auto_measure_params_frame, width=10, textvariable=self.auto_cur_dy).grid(column=3, row=3, sticky=(W, E))
        ttk.Button(self.auto_measure_params_frame, text="Set", command=self.set_auto_dy).grid(column=4, row=3, sticky=W)
        
        ttk.Separator(self.auto_measure_params_frame, orient=HORIZONTAL).grid(column=0, row=4, sticky='we', pady=10, padx=20, columnspan=5)
        
        self.auto_measurement_type = StringVar()
        self.auto_measurement_type.set('power')
        
        ttk.Label(self.auto_measure_params_frame, text="Data to record: ").grid(column=0, row=5, sticky=E)
        ttk.Radiobutton(self.auto_measure_params_frame, text="Power only", variable=self.auto_measurement_type, value='power').grid(column=1, row=5, sticky=W)
        ttk.Radiobutton(self.auto_measure_params_frame, text="Luna scan only", variable=self.auto_measurement_type, value='luna').grid(column=1, row=6, sticky=W)
        ttk.Radiobutton(self.auto_measure_params_frame, text="Power and Luna scan", variable=self.auto_measurement_type, value='both').grid(column=1, row=7, sticky=W)
        
        self.auto_power_filename = StringVar()
        self.auto_power_filename.set('power_measurement_data.csv')
        
        self.auto_luna_prefix = StringVar()
        self.auto_luna_prefix.set('luna_data_prefix')
        
        self.auto_data_folder = StringVar()
        self.auto_data_folder.set('auto_measurement_folder')
        
        ttk.Label(self.auto_measure_params_frame, width=20, text="Data folder name...").grid(column=2, row=5, sticky=W)
        ttk.Entry(self.auto_measure_params_frame, width=40, textvariable=self.auto_data_folder).grid(column=3, row=5, sticky=(W, E), columnspan=2)
        
        ttk.Label(self.auto_measure_params_frame, width=20, text="Power data filename...").grid(column=2, row=6, sticky=W)
        ttk.Entry(self.auto_measure_params_frame, width=40, textvariable=self.auto_power_filename).grid(column=3, row=6, sticky=(W, E), columnspan=2)
        
        ttk.Label(self.auto_measure_params_frame, width=20, text="Luna data prefix...").grid(column=2, row=7, sticky=W)
        ttk.Entry(self.auto_measure_params_frame, width=40, textvariable=self.auto_luna_prefix).grid(column=3, row=7, sticky=(W, E), columnspan=2)
        
        ttk.Button(self.auto_measure_params_frame, text="Start Auto Measurement Routine", command=self.start_auto_measurement).grid(column=3, row=8, sticky=W, columnspan=2, pady=10)
        
    def set_auto_nx(self, *args):
        try:
            self.auto_cur_nx.set(int(self.auto_nx.get()))    
        except ValueError():
            pass
        
    def set_auto_ny(self, *args):
        try:
            self.auto_cur_ny.set(int(self.auto_ny.get()))    
        except ValueError():
            pass
        
    def set_auto_dx(self, *args):
        try:
            self.auto_cur_dx.set(float(self.auto_dx.get()))    
        except ValueError():
            pass
        
    def set_auto_dy(self, *args):
        try:
            self.auto_cur_dy.set(float(self.auto_dy.get()))    
        except ValueError():
            pass
        
    def start_auto_measurement(self, *args):
        try:
            if str(self.auto_measurement_type.get())=='power':
                if self.stageON==False:
                    messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')
                else:
                    """
                    Perform a power only auto-measurement
                    """
                    directory = str(self.auto_data_folder.get())
                    if not os.path.exists(directory):
                        os.makedirs(directory)
                        
                    dx, nx, dy, ny = float(self.auto_cur_dx.get()), int(self.auto_cur_nx.get()), float(self.auto_cur_dy.get()), int(self.auto_cur_ny.get())
                    x0, y0 = self.pc.query_position()
                    
                    x_list, y_list = [], []
                    x_coords, y_coords = [], []
                    power_list = []
                    
                    device_number_list=[]
                    device_number = 0
                    
                    prev_x, prev_y = x0, y0
                    
                    """ First generate the list of points """
                    pos_list = []
                    for i in range(int(nx)):
                        for j in range(int(ny)):
                            if i%2==0: #even, go up!
                                pos_list.append((i, j))
                            elif i%2==1: #odd, go down!
                                pos_list.append((i, ny-1-j))
                    
                    for (i,j) in pos_list:
                        # determine next position of current device in absolute coords
                        x, y = x0+i*dx, y0+j*dy
                        
#                            print("Currently at (x, y) = "+str((prev_x, prev_y)))
                        
                        move_x, move_y = x-prev_x, y-prev_y
                        
                        #Not going to use prev_x, so move prev_y to next value
                        prev_x, prev_y = x, y
                        
                        print("Move to next device by: "+str((move_x, move_y)))
                        
                        self.pc.move_stage_relative(move_x, move_y)
                        maxpower, (max_pos_x, max_pos_y) = self.spiral_search()
                        self.pc.move_stage(max_pos_x, max_pos_y)
                        self.pc.close_spiral_plot()
                        
                        maxpower, (max_pos_x, max_pos_y) = self.spiral_search_fine()
                        self.pc.move_stage(max_pos_x, max_pos_y)
                        self.pc.close_spiral_plot()
                        
                        x_list.append(i)
                        y_list.append(j)
                        x_coords.append(x)
                        y_coords.append(y)
                        power_list.append(maxpower)
                
                        df = pd.DataFrame({"Device x- index" : np.array(x_list),
                                           "Device y- index" : np.array(y_list),
                                           "Device x-coordinate" : np.array(x_coords),
                                           "Device y-coordinate" : np.array(y_coords),
                                           "Power [W]" : np.array(power_list)})
                        df.to_csv(directory + '\\' + str(self.auto_power_filename.get()), index=False)  
                        
                        # Do some plotting in real-time
                        device_number += 1
                        device_number_list.append(device_number)
                        
                        plt.close()
                        plt.figure()
                        plt.plot(device_number_list, power_list, 'bo-')
                        plt.title("Auto-Measurement: max power seen at each device")
                        plt.xlabel('Device number')
                        plt.ylabel('Power [W]')
                        plt.pause(0.0001)
                        plt.show()
                            
                
            elif str(self.auto_measurement_type.get())=='luna':
                if self.lunaON==False:
                    messagebox.showinfo(message='Please check that the LUNA OVA5000 is powered on, connected, and calibrated then press "Connect to LUNA OVA5000" prior to attempting scan.')
                elif self.stageON==False:
                    messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')
                else:
                    """
                    Perform a luna-scan only measurement
                    """
                    directory = str(self.auto_data_folder.get())
                    prefix = str(self.auto_luna_prefix.get())
                    if not os.path.exists(directory):
                        os.makedirs(directory)
                    
                    dx, nx, dy, ny = float(self.auto_cur_dx.get()), int(self.auto_cur_nx.get()), float(self.auto_cur_dy.get()), int(self.auto_cur_ny.get())
                    x0, y0 = self.pc.query_position()
                    
                    x_list, y_list = [], []
                    x_coords, y_coords = [], []
                    
                    prev_x, prev_y = x0, y0
                                        
                    """ First generate the list of points """
                    pos_list = []
                    for i in range(int(nx)):
                        for j in range(int(ny)):
                            if i%2==0: #even, go up!
                                pos_list.append((i, j))
                            elif i%2==1: #odd, go down!
                                pos_list.append((i, ny-1-j))
                    
                    for (i,j) in pos_list:
                        # determine position of current device in absolute coords
                        x, y = x0+i*dx, y0+j*dy
                        
                        move_x, move_y = x-prev_x, y-prev_y
                        
                        #Not going to use prev_x, so move prev_y to next value
                        prev_x, prev_y = x, y
                        
                        self.pc.move_stage_relative(move_x, move_y)
                        maxpower, (max_pos_x, max_pos_y) = self.spiral_search()
                        self.pc.move_stage(max_pos_x, max_pos_y)
                        self.pc.close_spiral_plot()
                        
                        maxpower, (max_pos_x, max_pos_y) = self.spiral_search_fine()
                        self.pc.move_stage(max_pos_x, max_pos_y)
                        self.pc.close_spiral_plot()
                        
                        self.luna.close_plot() #Close previous plot
                        
                        self.luna_scan()
                        
                        cur_filename = directory + '\\' + prefix + '_x' + str(i) + '_y' + str(j) + '.csv'
                        self.luna.save_scan_data(filename=cur_filename)
                        
                        
                        x_list.append(i)
                        y_list.append(j)
                        x_coords.append(x)
                        y_coords.append(y)
                        
                        df = pd.DataFrame({"Device x- index" : np.array(x_list),
                                           "Device y- index" : np.array(y_list),
                                           "Device x-coordinate" : np.array(x_coords),
                                           "Device y-coordinate" : np.array(y_coords)})
                        df.to_csv(directory + '\\scan_coordinates.csv', index=False)  
                            
                    
            elif str(self.auto_measurement_type.get())=='both':
                if self.lunaON==False:
                    messagebox.showinfo(message='Please check that the LUNA OVA5000 is powered on, connected, and calibrated then press "Connect to LUNA OVA5000" prior to attempting scan.')
                elif self.stageON==False:
                    messagebox.showinfo(message='Please check that the PI stages and AQ2140 power meter are powered on and connected, then press "Connect to PICtester" prior to moving stages.')
                else:
                    """
                    Perform a measurement where both are recorded
                    """
                    directory = str(self.auto_data_folder.get())
                    prefix = str(self.auto_luna_prefix.get())
                    if not os.path.exists(directory):
                        os.makedirs(directory)
                        
                    dx, nx, dy, ny = float(self.auto_cur_dx.get()), int(self.auto_cur_nx.get()), float(self.auto_cur_dy.get()), int(self.auto_cur_ny.get())
                    x0, y0 = self.pc.query_position()
                    
                    x_list, y_list = [], []
                    x_coords, y_coords = [], []
                    power_list = []
                    
                    device_number_list=[]
                    device_number = 0
                    
                    prev_x, prev_y = x0, y0
                                        
                    """ First generate the list of points """
                    pos_list = []
                    for i in range(int(nx)):
                        for j in range(int(ny)):
                            if i%2==0: #even, go up!
                                pos_list.append((i, j))
                            elif i%2==1: #odd, go down!
                                pos_list.append((i, ny-1-j))
                    
                    for (i,j) in pos_list:
                        # determine position of current device in absolute coords
                        x, y = x0+i*dx, y0+j*dy
                        
                        move_x, move_y = x-prev_x, y-prev_y
                        
                        #Not going to use prev_x, so move prev_y to next value
                        prev_x, prev_y = x, y
                        
                        self.pc.move_stage_relative(move_x, move_y)
                        maxpower, (max_pos_x, max_pos_y) = self.spiral_search()
                        self.pc.move_stage(max_pos_x, max_pos_y)
                        self.pc.close_spiral_plot()
                        
                        maxpower, (max_pos_x, max_pos_y) = self.spiral_search_fine()
                        self.pc.move_stage(max_pos_x, max_pos_y)
                        self.pc.close_spiral_plot()
                        
                        self.luna.close_plot() #Close previous plot
                        
                        plt.close()
                        self.luna_scan()
                        
                        cur_filename = directory + '\\' + prefix + '_x' + str(i) + '_y' + str(j) + '.csv'
                        self.luna.save_scan_data(filename=cur_filename)
                        
                        x_list.append(i)
                        y_list.append(j)
                        x_coords.append(x)
                        y_coords.append(y)
                        power_list.append(maxpower)
                
                        df = pd.DataFrame({"Device x- index" : np.array(x_list),
                                           "Device y- index" : np.array(y_list),
                                           "Device x-coordinate" : np.array(x_coords),
                                           "Device y-coordinate" : np.array(y_coords),
                                           "Power [W]" : np.array(power_list)})
                        df.to_csv(directory + '\\' + str(self.auto_power_filename.get()), index=False)  
                        
                        # Do some plotting in real-time
                        device_number += 1
                        device_number_list.append(device_number)
                        
                        plt.figure()
                        plt.plot(device_number_list, power_list, 'bo-')
                        plt.title("Auto-Measurement: max power seen at each device")
                        plt.xlabel('Device number')
                        plt.ylabel('Power [W]')
                        plt.show()
                
        except ValueError():
            pass


if __name__ == '__main__':
    root = Tk()
    puma_gui = PUMA_GUI(root)
    
    #Event loop
    root.mainloop()
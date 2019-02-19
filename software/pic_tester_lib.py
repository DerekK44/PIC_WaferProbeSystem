# -*- coding: utf-8 -*-
"""
Created on Sat Sep 15 15:24:32 2018

@author: Derek Kita, 2018
"""

from __future__ import print_function
from pipython import GCSDevice, pitools
import visa
import time
import matplotlib as mpl
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import operator
import pandas as pd

UM2MM = 0.001 #conversion between um to mm

class PICtester:
    
    def __init__(self, PD_GPIB='GPIB0::17::INSTR'):
        self.startup_motors()
        self.startup_photodetector(PD_GPIB)
        
        self.angle_offset = 0.0 #DEFAULT TO ZERO, NO ROTATION
        
        self.spiral_data = [[], [], []]
        
    def close(self):
        self.xmotor.close()
        self.ymotor.close()
        self.pd.close()

    def startup_motors(self):
        """
        Function for connecting to the PhysikeInstrumente XY stages
        
        Returns
        """
        # C-863 controller with device ID 1, this is the master device
        # C-863 controller with device ID 2
        
        master_SN = '0185500606' # serial number of the master controller (the one connected to the PC)
        CONTROLLERNAME = 'C-863.11'
        STAGES = ['M-406.6DG']  # connect stages to axes
        REFMODES = ['FRF']  # reference the connected stages
        self.axis='1' #All M-406.6DG stages are linear, meaning they only have 1 axis
        
        """Connect two controllers on a daisy chain."""
        self.xmotor = GCSDevice(CONTROLLERNAME)
        self.xmotor.OpenUSBDaisyChain(description=master_SN)
        daisychainid = self.xmotor.dcid
        self.xmotor.ConnectDaisyChainDevice(1, daisychainid)
        
        self.ymotor = GCSDevice(CONTROLLERNAME)
        self.ymotor.ConnectDaisyChainDevice(2, daisychainid)
        print('\n{}:\n{}'.format(self.xmotor.GetInterfaceDescription(), self.xmotor.qIDN()))
        print('\n{}:\n{}'.format(self.ymotor.GetInterfaceDescription(), self.ymotor.qIDN()))
        # Show the version info which is helpful for PI support when there
        # are any issues.
        if self.xmotor.HasqVER():
            print('version info:\n{}'.format(self.xmotor.qVER().strip()))
            
        print('initialize connected stages...')
        pitools.startup(self.xmotor, stages=STAGES, refmodes=REFMODES)
        pitools.startup(self.ymotor, stages=STAGES, refmodes=REFMODES)
        
        self.xmin = self.xmotor.qTMN()[self.axis]
        self.xmax = self.xmotor.qTMX()[self.axis]
        self.xcur = self.xmotor.qPOS()[self.axis]
        
        self.ymin = self.ymotor.qTMN()[self.axis]
        self.ymax = self.ymotor.qTMX()[self.axis]
        self.ycur = self.ymotor.qPOS()[self.axis]
        
        print("curpos = "+str((self.xcur, self.ycur)))
        
        """ Set the origin (center of the stage) to be (0,0) """
        self.origin = {'x':(self.xmax-self.xmin)/2.0, 
                       'y':(self.ymax-self.ymin)/2.0}
        
    def move_stage(self, xnew, ynew):
        """ Move to the new position (xnew, ynew) 
        IN UNITS OF MICRONS """
        
        #Send command to motor, accounting for origin and micron to millimeter conversion
        self.xmotor.MOV(self.axis, (UM2MM*xnew)+self.origin['x'])
        self.ymotor.MOV(self.axis, (UM2MM*ynew)+self.origin['y'])
        
        self.xcur = (UM2MM*xnew)+self.origin['x']
        self.ycur = (UM2MM*ynew)+self.origin['y']
        
        #Wait for move to complete
        pitools.waitontarget(self.xmotor, axes=self.axis)
        pitools.waitontarget(self.ymotor, axes=self.axis)
        
    def move_stage_relative(self, xmove, ymove):
        """ Move to new position with relative offset (xmove, ymove)
        IN UNITS OF MICRONS """
        
        #First account for rotation
        xmove_r = xmove*np.cos(self.angle_offset) - ymove*np.sin(self.angle_offset)
        ymove_r = xmove*np.sin(self.angle_offset) + ymove*np.cos(self.angle_offset)
        
        #Next, send command to motor, accounting for origin and micron to millimeter conversion
        self.xmotor.MOV(self.axis, (UM2MM*xmove_r)+self.xcur)
        self.ymotor.MOV(self.axis, (UM2MM*ymove_r)+self.ycur)
        
        self.xcur = (UM2MM*xmove_r)+self.xcur
        self.ycur = (UM2MM*ymove_r)+self.ycur
        
        #Wait for move to complete
        pitools.waitontarget(self.xmotor, axes=self.axis)
        pitools.waitontarget(self.ymotor, axes=self.axis)
        
    def query_position(self):
        """ Get the current position of the stages """
        motor_position = (self.xmotor.qPOS(self.axis)[self.axis], self.ymotor.qPOS(self.axis)[self.axis])
        return ((motor_position[0]-self.origin['x'])/UM2MM, (motor_position[1]-self.origin['y'])/UM2MM)
        
    def startup_photodetector(self, GPIB):
        rm = visa.ResourceManager()
        self.pd = rm.open_resource(GPIB)
        
    def parse_AQ2140_output(self, output):
        for i in range(len(output)):
            letter = output[i]
            if i==1:
                if letter=='Z' or letter=='U' or letter=='O' or letter=='R':
                    """ Wait a bit then measure again """
#                    print("Measurement is out of range or during range change.  Try again soon..")
                    return "WAIT"
                if letter=='C':
                    print("Measurement is during a reference value.  Not sure what this means, check manual..")
                    return "WAIT"
            if i==2:
                if letter=='2' or letter=='3' or letter=='A' or letter=='B' or letter=='C':
                    raise ValueError("Measurement using units other than Watts.  Please change this on the controller.")
            if i==3:
                if letter=='O':
                    multiplier=1.0 #Watts
                if letter=='P':
                    multiplier=1E-3
                if letter=='Q':
                    multiplier=1E-6
                if letter=='R':
                    multiplier=1E-9
                if letter=='S':
                    multiplier=1E-12
                if letter=='U' or letter=='V' or letter=='W':
                    raise ValueError("Measurement unit is either dBm, dB, or not specified. Please change on controller.")
                
            if letter==' ':
                """ Finished with the formatting, now get the number """
                return multiplier*float(output[i:])
    
    def get_power(self):
        """ Get the current power incident upon the photodetector (in Watts) """
        output = self.parse_AQ2140_output(self.pd.query("OD1"))
        while output == "WAIT":
            # Wait a bit, then query again to see if signal's in an OK range
            time.sleep(0.5)
            output = self.parse_AQ2140_output(self.pd.query("OD1"))
            
        return output
    
    def spiral_search(self, radius_max=15.0, arm_dist=3.0, dtheta=2*np.pi/8.0, show_plot=True):
        """ 
        Perform a search for optimal signal.  Search path is an Archimedean spiral
        that obeys r = b*theta, where b is defined by b = arm_dist/2*Pi
        
        After searching, the stage should *move* to the position of the largest signal,
        return the corresponding X,Y position,
        and *plot* the intensity as a function of X & Y positions during the search (in real-time)
        """
        b = arm_dist/(2*np.pi)
        
        # First create a list of all the points
        theta = 0.0
        r = 0.0
        x0, y0 = self.query_position() #current position of the stage in absolute coords
        pt_list = [(x0+r*np.cos(theta), y0+r*np.sin(theta))]
        while r <= radius_max:
            r = b*theta
            theta = theta + dtheta
            pt_list.append((x0+r*np.cos(theta), y0+r*np.sin(theta)))
        pt_list = pt_list[::-1] #reverse so it goes from outwards to inwards
            
        print("Sweeping through all "+str(len(pt_list))+" spiral values, centered at "+str(self.query_position())+".")
        
        # Setup plotting
        mpl.rcParams['legend.fontsize'] = 10

        if show_plot:
            fig = plt.figure()
            ax = fig.gca(projection='3d')
        
        # Now for each coordinate in the list, do a search:
        xpos, ypos, powers = [], [], []
        for coord in pt_list:
            self.move_stage(coord[0], coord[1])
            power = self.get_power()
            cur_coords = self.query_position()
            xpos.append(cur_coords[0])
            ypos.append(cur_coords[1])
            powers.append(power)
            
            if show_plot:
                ax.clear()
                ax.set_xlim(x0-radius_max, x0+radius_max)
                ax.set_ylim(y0-radius_max, y0+radius_max)  
                ax.scatter3D(xpos, ypos, powers, c=powers, cmap='jet')
                ax.plot3D(xpos, ypos, powers, 'gray')
                ax.set_xlabel('X [um]')
                ax.set_ylabel('Y [um]')
                ax.set_zlabel('Power [W]')
                plt.show()
                plt.pause(0.0001) #Note this correction
            
        print("Spiral sweep finished.")
        
        max_index, max_value = max(enumerate(powers), key=operator.itemgetter(1))
        print("Max value of "+str(max_value)+"W found at location "+str(pt_list[max_index]))
        
        self.move_stage(pt_list[max_index][0], pt_list[max_index][1])
        print("Now stage is at location "+str(self.query_position()))
        
        self.spiral_data = [xpos, ypos, powers]
        
        return max_value, pt_list[max_index]
    
    def close_spiral_plot(self):
        plt.close()
        plt.clf()
    
    def save_spiral_data(self, filename="spiral_data.csv"):
        df = pd.DataFrame({"x position [um]" : np.array(self.spiral_data[0]),
                           "y position [um]" : np.array(self.spiral_data[1]),
                           "power [W]" : np.array(self.spiral_data[2])})
        df.to_csv(filename, index=False)  
    
    def power_scan(self, xstep, xnum, ystep, ynum, threshold=0.0):
        """ Measure the power transmission through a large number of devices
        assembled in a grid.  Assumes that you have already aligned to the 
        LOWER RIGHT device
            
        arguments:
            xstep = step-size along the x-direction in microns
            xnum = number of devices along the x-direction
            ystep = step-size along the y-direction in microns
            ynum = number of devices along the y-direction
            
        kwargs:
            threshold (optional) = a value that the maximum signal must be *greater* than
            in order to proceed to the next device
            
        returns:
            an array of size xnum x ynum with the *maximum* detected power for each device
            
        """
        power_list = np.zeros((xnum, ynum))
        x0, y0 = self.query_position()
        
        for i in range(int(xnum)):
            for j in range(int(ynum)):
                # determine position of current device in absolute coords
                x, y = x0+i*xstep, y0+j*ystep
                
                self.move_stage(x, y)
                
                next_device=False #initialize
                while next_device==False:
                    # Coarse alignment
                    # after this, stage is at new maximum and returns the new max PD value
                    cur_max, max_pos = self.spiral_search(radius_max=50.0, 
                                                 arm_dist=5.0,
                                                 dtheta=2.0*np.pi/8.0)
                    
                    # Fine alignment
                    # after this, stage is at new maximum and returns the new max PD value
                    cur_max, max_pos = self.spiral_search(radius_max=10.0, 
                                                 arm_dist=1.0,
                                                 dtheta=2.0*np.pi/8.0)
                    
                    if cur_max > threshold:
                        # Continue on to next device, else iteratively look for new max
                        next_device=True
                        power_list[i][j] = cur_max
        
        return power_list
        
if __name__ == '__main__':
    pc = PICtester()
    
    pc.move_stage(0,0)
    pc.spiral_search(radius_max=500, arm_dist=40)
    
#    move_points = [(0,0),
#                   (0.5,0.5),
#                   (1.0, 1.0),
#                   (1.5,1.5),
#                   (2.0,2.0),
#                   (2.5,2.5),
#                   (0.0, 0.0)]
    
#    for coords in move_points:
#        start_time = time.time()
#        pc.move_stage(2*coords[0], 2*coords[1])
#        print(pc.get_power())
#        print(pc.query_position())
#        print("--- %s seconds to complete ---" % (time.time() - start_time))
    
#    pc.move_stage(1000.0, 1000.0)
#    print(pc.query_position())
#    pc.move_stage(0.0, 0.0)
#    print(pc.query_position())
    
#    start_time = time.time()
#    print(pc.get_power())
#    print("--- %s seconds to complete ---" % (time.time() - start_time))lll
    
    pc.close()
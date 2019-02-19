# -*- coding: utf-8 -*-
"""
Created on Wed Sep 12 19:54:26 2018

@author: PUMA
"""
import visa
import matplotlib.pyplot as plt
import time
import numpy as np
import pandas as pd

class Luna:
    
    def __init__(self, GPIB='GPIB0::8::INSTR'):
        self.startup_luna(GPIB)
        
        self.min_loss = [] #empty until measured for the first time
        self.max_loss = []
        self.wl_list = []
        
    def close(self):
        self.luna.close()
        
    def startup_luna(self, GPIB):
        rm = visa.ResourceManager()
        self.luna = rm.open_resource(GPIB)
        
        self.luna.timeout = 120000
        
        print(self.luna.query("*IDN?"))

        while self.luna.query("SYST:WARM?") != '1':
            print("LUNA system is still warming up...")
            time_remaining = float(luna.query("SYST:WTIM?"))
            print(str(time_remaining)+" minutes remaining")
            time.sleep(time_remaining*60.0)
        print("Luna is at operating temperature.")
        
        #Turn on the laser
        self.luna.write("SYST:LASE 1")
        if self.luna.query("SYST:LASE?")=='1':
            print("Luna laser is ON")
        else:
            print("Warning! Luna laser is OFF.")
            
#        self.set_source_wavelength(1550.0)
#        self.turnon_light_source()
#        print("Luna is on and at 1550.0 nm")
        
    def configure_scan_parameters(self, 
                                  center_wl=None, 
                                  start_wl=None,
                                  scan_range=None
                                  ):
        """ Configure the scan parameters
        center_wl = center wavelength in NANOMETERS
        start_wl = start wavelength in NANOMETERS
        """
        if center_wl != None:
            self.luna.write("CONF:CWL "+str(center_wl))
            
        if start_wl != None:
            self.luna.write("CONF:STAR "+str(start_wl))
            
        if scan_range != None:
            self.luna.write("CONF:RANG "+str(scan_range))
            
    def find_DUT(self):
        """ Automatically find the DUT and set it
        """
        self.luna.write("CONF:DUTL")
        
    def get_DUT(self):
        return self.luna.query("CONF:DUTL?")
    
    def set_source_wavelength(self, wavelength):
        """ Sets the luna to "source" mode and gives it the wavelength below
        wavelength should be specified in NANOMETERS
        """
        self.luna.write("CONF:WAVE "+str(wavelength))
    
    def turnon_light_source(self):
        """ Should first set the source wavelength using set_source_wavelength
        """
        self.luna.write("CONF:SOUR 0")
    
    def scan(self, plot=True, meas_type=11):
        """ Performs a scan using the previously configured values.
        
        By default, meas_type is set to "11", which means that the min/max loss
        will be returned and plotted.
        
        Returns an array with the measured data and plots the values
        """
        self.luna.write("CONF:SOUR 1") #Close the source switch
        
        if self.luna.query("SYST:RDY?")=='1':
            print("System ready and scanning...")
            self.luna.write("SCAN")
        else:
            print("Warning! System is not ready for a scan...")
            print("SYST:RDY? status = "+str(self.luna.query("SYST:RDY?")))
            return None
        
        systerr = self.luna.query("SYST:ERR?")
        if systerr !='0':
            print("Warning!  System error occured during scan.")
            print("Luna error code: "+str(systerr))
            description = self.luna.query("SYST:ERRD?")
            print("Description: "+str(description))
            
        output_size = int(self.luna.query("FETC:FSIZ?"))
        output = self.luna.query("FETC:MEAS? "+str(meas_type))
        meas_data = [float(s) for s in output.split('\r')[:-1]]
        min_loss, max_loss = meas_data[0:len(meas_data)//2], meas_data[len(meas_data)//2:]
        
        freq_data = self.luna.query("FETC:FREQ?").split(",")
        freq_list = [float(freq_data[0]) - i*float(freq_data[1]) for i in range(output_size)]
        wl_list = [299792458.0/freq_list[i] for i in range(len(freq_list))][::-1]
        
        if plot==True:
            plt.figure()
            plt.plot(wl_list, min_loss, 'b', label='min loss')
            plt.plot(wl_list, max_loss, 'r', label='max loss')
            plt.xlabel("wavelength [nm]")
            plt.ylabel("loss [dB]")
            plt.show()
            
        self.wl_list = wl_list
        self.min_loss, self.max_loss = min_loss, max_loss
        
        # Return luna to source mode
#        self.turnon_light_source()
        
    def close_plot(self):
        plt.close()
        plt.clf()
        
    def save_scan_data(self, filename='data.csv'):
        df = pd.DataFrame({"wavelength [nm]" : np.array(self.wl_list),
                           "min loss [dB]" : np.array(self.min_loss),
                           "max loss [dB]" : np.array(self.max_loss)})
        df.to_csv(filename, index=False)        
        
    def query_scan_parameters(self):
        """ Prints all the scan parameters """
        print("Center wavelength = "+str(self.luna.query("CONF:CWL?")))
        print("Start wavelength = "+str(self.luna.query("CONF:STAR?")))
        print("End wavelength = "+str(self.luna.query("CONF:END?")))
        print("Scan range = "+str(self.luna.query("CONF:RANG?")))
        if self.luna.query("CONF:TEST?")=='0':
            print("Measurement type = 'reflection'")
        elif self.luna.query("CONF:TEST?")=='1':
            print("Measurement type = 'transmission'")

if __name__ == '__main__':
    luna = Luna()
    
    luna.configure_scan_parameters(center_wl=1552.0,
                                   scan_range=20.0)
    luna.query_scan_parameters()
    
    luna.scan()
    
    luna.save_scan_data(filename="data_test.csv")
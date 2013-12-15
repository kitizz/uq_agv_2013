#!/usr/bin/env python

import threading
import serial
import string
from Queue import Queue
from multiprocessing import Lock
import os
from serial.tools import list_ports
from serial.serialutil import SerialException


class GPSData(object):

    
    def __init__(self,):
        self.portlist = []
        self.list_serial_ports();
        
        self.serial_open = False
        self.queue = Queue()
        
        #VTG STRINGS
        self.speedkhm = 0
        self.speedknots = 0
        self.headingM = 0
        self.headingT = 0
        
        #GGA STRINGS
        self.time = 0
        self.latitude = 0
        self.longitude = 0
        self.latitude_D = 'N'
        self.longitude_D = 'S'
        self.quality = 0
        self.satelliteno = 0
        self.HDOP = 0
        self.altitude = 0
        self.HOGAWGS84E = 0
        self.time_since_DGPS_update = 0
        self.DGPS_reference_ID = 0
        
        self.lock = threading.Lock()
        #Start GPS Reading Thread;
        #self.ser = serial.Serial(buadrate = 19200, port = 'COM12')
        
        #thread = threading.Thread(target=self.read_from_port, args=(serial_port))
        #thread.daemon = True
        #thread.start()
        
    def open_port(self, prefix='/dev/ttyACM', ports=[], baud=19200):
        """
        Opens a port either by automatically selecting from avaliable ports or by
        taking a manual COM selection
       
        turns serial_open based on success
        """

        if not ports:
            ports = range(3)

        success = False
        for port in ports:
            dev = prefix + str(port)
            try:
                self.serial = serial.Serial(dev, baud, timeout=2)
                self.serial_open = True
            except:
                print "Failed to open:", dev, "Attempting next port..."
                self.serial_open = False
            else:
                print "Succesfully opened:", dev
                break

        # #If manual COM port
        # if(serialport == 'COM0'):
        #     try:
        #         self.serial = serial.Serial(baudrate = 19200, port = serialport)
        #         self.serial_open = True
        #     except SerialException:
        #         self.serial_open = False
        #         print "Failed to open MANUAL gps serial port"
        #     except:
        #         print "Some other shit failed"
        # #If Automatic
        # else:
        #     self.list_serial_ports()
        #     if not len(self.portlist):
        #         print "No Ports Detected"
        #     elif len(self.portlist) == 1:
        #         self.serial = serial.Serial(baudrate = 19200, port = self.portlist[0])
        #         self.serial_open = True
        #         print "Opened port on " + self.portlist[0]
        #     elif len(self.portlist) > 1:
        #         print "Callum hasn't added the code for automatic port swtiching"
        #         self.serial_open = False

        if self.serial_open:
            print "GPS COM port opened"
            thread = threading.Thread(target=self.read_from_port, args=(self.serial,))
            #thread.daemon = True
            thread.start()
            #print threading.active_count()
        else:
            print "GPS COM port connect failed"
        
        
    def handle_data(self, data):
        """
        Determines if a VTG or GGA String and loads into memory
        """
        #print data[0]
        if data[0] == '$':
            if data.find('GPVTG') != -1:
                #print data.find("GPVTG")
                #print "it be VTG"
                data2 = [i.strip() for i in data.split(',')]
                self.lock.acquire()
                self.headingM = data2[1]
                self.headingT = data2[3]
                self.speedkhm = data2[5]
                self.speedknots = data2[7]
                #print "banans"
                self.lock.release()
                
            elif data.find("GPGGA") != -1:
                #print "it be GGA" 
                data2 =[i.strip() for i in data.split(',')]
                self.lock.acquire()
                self.time = data2[1]
                self.latitude = data2[2]
                self.latitude_D = data2[3]
                self.longitude = data2[4]
                self.longitude_D = data2[5]
                self.quality = data2[6]
                self.satelliteno = data2[7]
                self.HDOP = data2[8]
                self.altitude = data2[9]
                self.HOGAWGS84E = data2[11]
                self.time_since_DGPS_update = data2[13]
                self.DGPS_reference_ID = data2[14]
                self.lock.release()
            else:
                print "GPS Error"
            
    def read_from_port(self, ser):
        """
        Reads from the serial port runs in thread
        """
        try:
            while True:
                reading = ser.readline()
                #print reading
                self.handle_data(reading)
                    #handle_data(reading)
        except KeyboardInterrupt:
            print "NIGAAS STOPPIN"
            #thread.stop()
        except:
            #print error.
            print "error 123"  

    def get_Speed(self):
        return self.x
    
    def list_serial_ports(self):
        """ 
        Stringlist of the avaliable COM Ports on computer
        """
        # Windows
        if os.name == 'nt':
            # Scan for available ports.
            available = []
            for i in range(256):
                try:
                    s = serial.Serial(i)
                    available.append('COM'+str(i + 1))
                    s.close()
                except serial.SerialException:
                    pass
            self.portlist = available
        else:
            # Mac / Linux
            return [port[0] for port in list_ports.comports()]
    
    def get_speed(self):
        self.lock.aquire()
        return self.speedkhm
        self.lock.release()
        
    def get_headingT(self):
        self.lock.aquire()
        return self.headingT
        self.lock.release()
    
    def get_headingM(self):
        self.lock.aquire()
        return self.headingM
        self.lock.release()
    
    def get_latitude(self):
        self.lock.aquire()
        return self.latitude
        self.lock.release()
        
    def get_longitude(self):
        self.lock.aquire()
        return self.longitude
        self.lock.release()
        
    def get_latitudeD(self):
        self.lock.aquire()
        return self.latitudeD
        self.lock.release()
            
    def get_longitudeD(self):
        self.lock.aquire()
        return self.longitudeD
        self.lock.release()
        
    def get_time(self):
        self.lock.aquire()
        return self.time


if __name__ == "__main__":
    #thread = threading.Thread(target=read_from_port, args=(serial_port,))
    #thread.start()
    gps = GPSData()
    gps.open_port('/dev/ttyUSB1')
    #for i in range(0,1000):
    #    print gps.time, gps.latitude, gps.longitude

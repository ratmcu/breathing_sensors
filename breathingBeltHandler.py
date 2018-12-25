'''
Created on Dec 25, 2018

@author: rajitha
'''
from godirect import GoDirect
import time
godirect = GoDirect(use_ble=False, use_usb=True)

import time
import threading
import queue
import csv
import numpy as np
import sys
import threading
import queue

from globalVars import *
from io import open

class CollectionThreadGDXRB(threading.Thread):
    def __init__(self, threadID, name, dataQueue=None, dataLock=None, stopEvent =  None):
        threading.Thread.__init__(self)
        self.name = name
        self.threadID = threadID
        self.stopEvent = stopEvent
        self.dataQueue = dataQueue
        self.dataLock = dataLock
        self.device = godirect.get_device()
        print(self.device)

        if self.device != None and self.device.open(auto_start=True):
        	self.sensors = self.device.get_enabled_sensors()
        	print("Connected to "+self.device.name)
        	# print("Reading 10 measurements")
        # 	for i in range(0,10):
        # 		if device.read():
        # 			for sensor in sensors:
        # 				print(sensor.sensor_description+": "+str(sensor.value))
        # 	device.stop()
        # 	device.close()
        # godirect.quit()
        print ('Beathing Belt {0} collection thread initialized'.format(self.name))

    def run(self):
            startTime = time.time()
            print ('Starting Beathing Belt {0} data collection'.format(self.name))
            while not self.stopEvent.is_set():
                currentTime = time.time()
                if self.device.read():
                    for sensor in self.sensors:
                        value = sensor.value
                self.dataLock.acquire()
                self.dataQueue.put([int((currentTime - startTime) * 1000)] + [value])
                self.dataLock.release()
            self.device.stop()
            self.device.close()

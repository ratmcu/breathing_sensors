'''
Created on Dec 25, 2018

@author: rajitha
'''
# !pip install vernierpygatt
from godirect import GoDirect
import time
godirect = GoDirect(use_ble=False, use_usb=True)

import time
import threading
import queue
import csv
import sys
import threading
import queue

from io import open

class GoDirectDivices():
    def __init__(self):
        self.devices = godirect.list_devices()
        self.device_list = []
        for device in self.devices:
            self.device_list.append(device)
            device.open(auto_start=False)
            print('found device: {0}'.format(device.name))
        # print('found devices: {0}'.format(godirect.list_devices()))

    def __del__(self):
        for device in self.devices:
        	device.stop()
        	device.close()
        godirect.quit()


class CollectionThreadGDXRB(threading.Thread):
    def __init__(self, threadID, name, dataQueue=None, dataLock=None, stopEvent =  None):
        threading.Thread.__init__(self)
        self.name = name
        self.threadID = threadID
        self.stopEvent = stopEvent
        self.dataQueue = dataQueue
        self.dataLock = dataLock
        self.devices = godirect.list_devices()
        self.device = None
        self.device_list = []
        for device in self.devices:
                self.device_list.append(device)
                device.open(auto_start=True)
                print('found device: {0}'.format(device.name))
                if device.name == self.name:
                        self.device = device
                device.stop()
                device.close()
        print('found devices: {0}'.format(godirect.list_devices()))

        if self.device != None and self.device.open(auto_start=True) and self.device.name == self.name:
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
        else:
                if self.device != None:
                        print('listed sensor {0} not found instead found {1}'.format(self.name,self.device.name))
                raise RuntimeError
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

class CollectionThreadGDXRBDummy(threading.Thread):
    def __init__(self, threadID, name, device, dataQueue=None, dataLock=None, stopEvent =  None):
        threading.Thread.__init__(self)
        self.name = name
        self.threadID = threadID
        self.stopEvent = stopEvent
        self.dataQueue = dataQueue
        self.dataLock = dataLock
        self.device = device
        self.device.open(auto_start=True)
        self.sensors = self.device.get_enabled_sensors()
        print ('Beathing Belt {0} collection thread initialized'.format(self.device.name))

    def run(self):
            startTime = time.time()
            print ('Starting Beathing Belt {0} data collection'.format(self.name))
            while not self.stopEvent.is_set():
                currentTime = time.time()
                if self.device.read():
                    for sensor in self.sensors:
                        value = sensor.value
                self.dataLock.acquire()
                # self.dataQueue.put([int((currentTime - startTime) * 1000)] + [value])
                self.dataQueue.put([currentTime] + [value])
                self.dataLock.release()
            self.device.stop()
            self.device.close()

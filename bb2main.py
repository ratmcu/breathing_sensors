'''
Created on Dec 25, 2018

@author: rajitha
'''

import time
import collections
import subprocess
import os
import csv
import math
import logging
import datetime

import multiprocessing
# from processBR import processBR
import threading
import queue
from breathingBeltHandlerHacked import CollectionThreadGDXRBDummy, GoDirectDivices

DIRECTORY_PATH = 'C:/workspace/data/RawBreathingBeltData/' + time.strftime(u"%Y%m%d") + '/'
def sensor_thread(device):
    name = device.name
    if not os.path.exists(DIRECTORY_PATH+name+'/'):
        os.makedirs(DIRECTORY_PATH+name+'/')
    bbeltDataLock = threading.Lock()
    stopEvent = threading.Event()
    bbeltDataQ = queue.Queue()
    bbeltThread = CollectionThreadGDXRBDummy(threadID = 1, name = name, device=device,
                                            dataQueue=bbeltDataQ, dataLock=bbeltDataLock,
                                            stopEvent = stopEvent)
    bbeltThread.start()
    bbeltDataDeck = collections.deque(maxlen=60*10)
    dataList = []
    t = threading.currentThread()
    while getattr(t, "do_run", True):
        if not bbeltDataQ.empty():
            bbeltDataLock.acquire()
            while not bbeltDataQ.empty():
                dataList.append(bbeltDataQ.get())
            bbeltDataLock.release()
        # print(len(dataList))
        bbeltDataDeck.extend(dataList)
        dataList = []
        if len(bbeltDataDeck) == 600:
            # startTime = time.time()
            with open(DIRECTORY_PATH+device.name+'/' + 'BREATHING_BELT_' + name +'_'+ time.strftime(u"%Y%m%d-%H%M%S") +'.csv', 'w') as csvFile:
                csvWriter = csv.writer(csvFile)
                for bbDataRow in bbeltDataDeck:
                    csvWriter.writerow(bbDataRow)
            bbeltDataDeck.clear()
            print ('Data from {0} saved...................'.format(name))
            # elapsedTime = time.time() - startTime
            # print ('Elapsed %f ms' % (elapsedTime * 1000))
    stopEvent.set()
    bbeltThread.join()

if __name__ == '__main__':
    godirect_devs = GoDirectDivices()
    sensor_threads = []
    for device in godirect_devs.device_list:
        sensor_threads.append(threading.Thread(target=sensor_thread, args = (device,)))
    try:
        for t in sensor_threads:
            t.start()
        while True:
            pass
    except KeyboardInterrupt:
        for t in sensor_threads:
            t.do_run = False
            t.join()

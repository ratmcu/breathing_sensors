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
import numpy as np
import scipy as sp
import scipy.stats
from ast import literal_eval
import logging
import datetime

import multiprocessing
# from processBR import processBR
import threading
import queue
from breathingBeltHandler import CollectionThreadGDXRB

import pandas as pd
DIRECTORY_PATH = 'C:/workspace/data/RawBreathingBeltData/' + time.strftime(u"%Y%m%d") + '/'
def main():
    if not os.path.exists(DIRECTORY_PATH):
        os.makedirs(DIRECTORY_PATH)
    name = 'GDX-RB 0K1002U5'
    bbeltDataLock = threading.Lock()
    stopEvent = threading.Event()
    bbeltDataQ = queue.Queue()
    bbeltThread = CollectionThreadGDXRB(threadID = 2, name = name, dataQueue=bbeltDataQ, dataLock=bbeltDataLock, stopEvent = stopEvent)
    bbeltThread.start()
    bbeltDataDeck = collections.deque(maxlen=60*10)
    dataList = []
    try:
        while True:
            if not bbeltDataQ.empty():
                bbeltDataLock.acquire()
                while not bbeltDataQ.empty():
                    dataList.append(bbeltDataQ.get())
                bbeltDataLock.release()
            # print(len(dataList))
            bbeltDataDeck.extend(dataList)
            dataList = []
            if len(bbeltDataDeck) == 600:
                startTime = time.time()
                with open(DIRECTORY_PATH + 'BREATHING_BELT_' + name +'_'+ time.strftime(u"%Y%m%d-%H%M%S") +'.csv', 'w') as csvFile:
                    csvWriter = csv.writer(csvFile)
                    for bbDataRow in bbeltDataDeck:
                        csvWriter.writerow(bbDataRow)
                bbeltDataDeck.clear()
                print ('Data saved...................')
                elapsedTime = time.time() - startTime
                print ('Elapsed %f ms' % (elapsedTime * 1000))
    except KeyboardInterrupt:
        stopEvent.set()
if __name__ == '__main__':
    main()

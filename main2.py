u'''
Created on Mar 24, 2017

@author: isuru, modified by rajitha to accomodate sensor multiplexing 12/24/2018
'''

from __future__ import with_statement
from __future__ import division
from __future__ import absolute_import
from __future__ import print_function

try:
    import rospy
    from std_msgs.msg import String
    from std_msgs.msg import Float32MultiArray
    from std_msgs.msg import Float32, Int32
    from sensor_msgs import msg
    ROS_AVAILABLE = True
except:
    print('ROS not found')
    ROS_AVAILABLE = False
ROS_AVAILABLE = False # let's keep it so, until we fix the main thread issue for ros node init

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
#from processBR import processBR
#import HttpThread
import threading
import queue
from globalVars import *
from radarHandler import CollectionThread,CollectionThreadX4
from main import RadarThread, MainClass
import pandas as pd

from colorPrint import bcolors
# from io import open
try:
    import matlab.engine
    MATLAB_AVAILABLE = True
except ImportError:
    print('Matlab Engine not found')
    MATLAB_AVAILABLE = False


if __name__ == '__main__':
    if ROS_AVAILABLE:
        rospy.init_node('realtimeBreathing_2')
    if not os.path.exists('results'):  # Save as JPEG in folder
        os.makedirs('results')
    if not os.path.exists('logs'):
        os.makedirs('logs')
    rd2 = MainClass(radarNumber = 2, port='COM5')
    rd_thread_2 = RadarThread(22,'sensorRadarThread2',object=rd2)
    rd_thread_2.start()
    rd_thread_2.join()

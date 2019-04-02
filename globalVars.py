from __future__ import division
from __future__ import absolute_import
import threading
import queue

#Global variables
radarDataLock = threading.Lock()
radarDataQ = queue.Queue()

#Radar parameters
# RADAR_RESOLUTION = 3.90625 / 1000 #X2
# RADAR_RESOLUTION = 51.8617 / 1000

#Processing options
WINDOW_LENGTH = 20 #length of time window in seconds
PROCESSING_INTERVAL = 1 #Process the data once every <PROCESSING_INTERVAL> seconds
MAX_FS = 20 #Maximum sampling frequency supported to maintain given Window length

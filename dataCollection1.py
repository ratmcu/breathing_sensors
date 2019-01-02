from __future__ import division

# import matplotlib
# matplotlib.use('Qt4Agg')

import rospy
from std_msgs.msg import Float32MultiArray, Float32, String, Int8
import Queue, time

from sensor_msgs.msg import Range

from sensor_msgs.msg import LaserScan

import os.path
import csv
import sys
import datetime

from std_msgs.msg import String

from collections import deque
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

date = datetime.datetime.now()
time = date.strftime("%Y-%m-%d-%A-%X").replace(':', '_')
folder_data='/mnt/c/workspace/data/radar/cleanData_radar_1'
file_radar = folder_data+'/DataRadar_1'+'_' + time+'.csv'
file_camera= folder_data+'/DataCam' + '_' + time+'.csv'

def cameraConCallBack(data):
    # rospy.loginfo(rospy.get_caller_id() +"rangeCon"+ data.data)
    print (data.header.stamp)
    print (data.time_increment)
    # # print(data.field_of_view)
    # # print(data.max_range)
    if os.path.isfile(file_camera):
        f = open(file_camera, 'a')
        writer = csv.writer(f)
        writer.writerow((data.header.stamp, data.time_increment ,data.scan_time ,data.angle_min , data.range_min))
    else:
        f = open(file_camera, 'a')
        writer = csv.writer(f)
        writer.writerow(('timestamp', 'posture', 'footRightPosY','footLeftPosY','lowest_y_point'))
        writer.writerow((data.header.stamp, data.time_increment ,data.scan_time ,data.angle_min , data.range_min))

def radarConCallBack(data):
    # rospy.loginfo(rospy.get_caller_id() +"rangeCon"+ data.data)
    print (data.header.stamp)
    print (data.range)
    print(data.field_of_view)
    print(data.max_range)

    if os.path.isfile(file_radar):
        f = open(file_radar, 'a')
        writer = csv.writer(f)
        writer.writerow((data.header.stamp, data.range, data.field_of_view, data.max_range))
    else:
        f = open(file_radar, 'a')
        writer = csv.writer(f)
        writer.writerow(('timestamp', 'radarRange', 'radarBR', 'radarActivity'))
        writer.writerow((data.header.stamp, data.range, data.field_of_view, data.max_range))

def listener():
        rospy.init_node('SData_C_R', anonymous=True)
        rospy.Subscriber("Cam_Data", LaserScan,cameraConCallBack)
        rospy.Subscriber("RadarRange_1", Range,radarConCallBack)
        rospy.spin()
if __name__ == '__main__':
    listener()

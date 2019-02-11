from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys
import time
import csv
import os

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

_kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)


def get_joint(joints, jointPoints, joint0):
        joint0State = joints[joint0].TrackingState;
        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint0State == PyKinectV2.TrackingState_Inferred):
            return [0.0, 0.0]
        # ok, at least one is good
        return [jointPoints[joint0].x, jointPoints[joint0].y]

def list_joints(person, joints, jointPoints):
    # Torso
    joint_list = ['person_'+str(person)]
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_Head);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_Neck);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_SpineShoulder);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_SpineMid);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_SpineShoulder);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_SpineBase);

    # Right Arm
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_ShoulderRight);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_ElbowRight);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_WristRight);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_HandRight);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_WristRight);

    # Left Arm
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_ShoulderLeft);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_ElbowLeft);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_WristLeft);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_HandLeft);

    # Right Leg
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_HipRight);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_KneeRight);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_AnkleRight);

    # Left Leg
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_HipLeft);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_KneeLeft);
    joint_list = joint_list + get_joint(joints, jointPoints, PyKinectV2.JointType_AnkleLeft);
    return joint_list
# here we will store skeleton data
_bodies = None
start_time  = time.time()
frames = []
PATH = 'C:/workspace/data/kinect/'+ time.strftime(u"%Y%m%d")
if not os.path.exists(PATH):
    os.makedirs(PATH)
PATH = PATH + '/'
while True:
    if _kinect.has_new_body_frame():
        _bodies = _kinect.get_last_body_frame()
    # --- draw skeletons to _frame_surface
    if time.time() - start_time > 60 and len(frames)!=0:
        with open( PATH+'kinect_frames'+time.strftime(u"%Y%m%d-%H%M%S")+'.csv', 'w') as csvFile:
            csvWriter = csv.writer(csvFile)
            print('writing csv..')
            for frame in frames:
                csvWriter.writerow(frame)
        frames = []
        start_time = time.time()
    if _bodies is not None:
        frame = []
        for i in range(0, _kinect.max_body_count):
            body = _bodies.bodies[i]
            if not body.is_tracked:
                continue

            joints = body.joints
            # convert joint coordinates to color space
            joint_points = _kinect.body_joints_to_color_space(joints)
            frame.append(list_joints(i,joints, joint_points))
            # print(list_joints(i,joints, joint_points))
        frames.append(frame)
        # with open( 'frames'+time.strftime(u"%Y%m%d-%H%M%S")+'_kinect.csv', 'w') as csvFile:

	            #self.draw_body(joints, joint_points, SKELETON_COLORS[i])
    time.sleep(1)

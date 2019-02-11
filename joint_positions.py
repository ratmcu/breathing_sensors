from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys
import time

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

_kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)


def get_joint(joints, jointPoints, joint0):
        joint0State = joints[joint0].TrackingState;
        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint0State == PyKinectV2.TrackingState_Inferred): 
            return (0.0, 0.0)
        # ok, at least one is good 
        return (jointPoints[joint0].x, jointPoints[joint0].y)

def list_joints(person, joints, jointPoints):
    # Torso
    joint_list = ['person'+str(person)]
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

while True:
    if _kinect.has_new_body_frame():
        _bodies = _kinect.get_last_body_frame()
    # --- draw skeletons to _frame_surface
    if _bodies is not None:
        for i in range(0, _kinect.max_body_count):
            body = _bodies.bodies[i]
            if not body.is_tracked:
                continue

            joints = body.joints
            # convert joint coordinates to color space
            joint_points = _kinect.body_joints_to_color_space(joints)
            print(i, list_joints(joints, joint_points))
            #self.draw_body(joints, joint_points, SKELETON_COLORS[i])
    time.sleep(1)

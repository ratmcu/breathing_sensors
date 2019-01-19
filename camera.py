import numpy as np
import cv2
import time
import os

DIRECTORY_PATH = '/home/isuru/combined_system/data/camera/' + time.strftime(u"%Y%m%d_%H_%M_%S") + '/'
# DIRECTORY_PATH = '/home/isuru/combined_system/data/camera/' + time.strftime("%a_%d_%b_%Y_%H_%M_%S") + '/'

if not os.path.exists(DIRECTORY_PATH):
    os.makedirs(DIRECTORY_PATH)
cap = cv2.VideoCapture(1)
file = open(DIRECTORY_PATH +'time.txt','w')
file.write(str(time.time()))
file.close()
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(DIRECTORY_PATH+'output.avi',fourcc, 20.0, (640,480))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        # frame = cv2.flip(frame,0)

        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()

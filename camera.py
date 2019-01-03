import numpy as np
import cv2
import time
import os

DIRECTORY_PATH = 'C:/workspace/data/video/' + time.strftime(u"%Y%m%d") + '/'
if not os.path.exists(DIRECTORY_PATH):
    os.makedirs(DIRECTORY_PATH)
cap = cv2.VideoCapture(0)

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

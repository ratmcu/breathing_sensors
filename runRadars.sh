#!/bin/bash

echo "establishing radar connection"
echo rdrmblc@srjn | sudo -S chmod 666 /dev/ttyS5 && stty -F /dev/ttyS5 sane 9600

echo rdrmblc@srjn | sudo -S chmod 666 /dev/ttyS3 && stty -F /dev/ttyS3 sane 9600
source activate snakes27tf
python3 /mnt/c/workspace/two_radar_system/radar_1_and_2/main.py

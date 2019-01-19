#!/bin/bash
echo rdrmblc@srjn | sudo -S chmod 666 /dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0 && stty -F /dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0 sane 9600

echo rdrmblc@srjn | sudo -S chmod 666 /dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0 && stty -F /dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0 sane 9600
source activate diogenesenv
export PYTHONPATH="/home/isuru/anaconda3/envs/diogenesenv/lib/python3.5/site-packages:$PYTHONPATH"
cd /home/isuru/combined_system/profbolic/radar_1_and_2
python3 camera.py


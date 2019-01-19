
##preparing conda environment
```
conda create -n diogenesenv python=3.5 anaconda 
conda install pip
conda install -c conda-forge opencv 
pip install opencv-python
```
python version 3.5 was used for for the developments

## installing intel realsense drivers
```
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
```
## installing python dependencies
```
pip install pyrealsense2
```

pymoduleconnector has to be installed from. 
[Module Connector Unix](https://www.xethru.com/community/resources/module-connector-unix.80/)

and then by running command in the **/pymoduleconnector/python35-x86_64-linux-gnu** directory in the extracted archive

```
python setup.py install
```

source the virtual environment before running
```
source activate diogenesenv
```
use the following command to add our virtual env library path to the python library path.(avoids possible confusions in importing libraries for modules)
```
export PYTHONPATH="/home/<username>/anaconda3/envs/diogenesenv/lib/python3.5/site-packages:$PYTHONPATH"
```

## Go direct Breathing Belts [GDX-RB](https://www.vernier.com/products/sensors/respiration-monitors/gdx-rb/)


```
pip install vernierpygatt
pip install godirect
pip install hidapi
```

## adding udev rules
```
sudo cp 99-usb-gdxrb.rules /etc/udev/rules.d/ 
```
make sure devices are plugges afterwards







#!/usr/bin/python

import rospkg
import rospy

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
path = rospack.get_path('calibrate_kinnect2') #+ '/data'
try:
    path = rospack.get_path('calibrate_kinnect') #+ '/data'
except:
    print("Could not find package calibrate_kinnect")
print(path)

#!/usr/bin/env python

import rospy

import baxter_interface

from sensor_msgs.msg import PointCloud2


def callback(msg):
    print ("Test")

def main():
    rospy.init_node('VR_PointCloud',anonymous=True)
    rospy.Subscriber('/kinect/depth/points', PointCloud2, callback)
    rospy.spin()



if __name__=='__main__':
	main()
#!/usr/bin/env python
import rospy
from pbdlib_manager.msg import PoseNameArray

def callback(data):
    header = data.header
    poses = dict(zip(data.names,data.poses))
    #rospy.loginfo(poses)
    rospy.loginfo(poses["camera_link"].position)
    #rospy.loginfo(poses["Headset"].position)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/HTCVive/Kinect", PoseNameArray, callback)
    #rospy.Subscriber("/HTCVive/RawData", PoseNameArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
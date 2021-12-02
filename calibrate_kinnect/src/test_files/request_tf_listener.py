#!/usr/bin/env python  

import rospy
import tf as tf_for_pos
import geometry_msgs.msg

def get_tf(a,b):
    max_wait= 50
    trans,rot = None,None
    rospy.init_node('baxter_tf_listener')
    listener = tf_for_pos.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(a,b,  rospy.Time(0))
            return True,trans,rot
        except (tf_for_pos.LookupException, tf_for_pos.ConnectivityException, tf_for_pos.ExtrapolationException):
            max_wait -=1
        if max_wait == 0:
            print("Could not find the tf of "+a+ " and " + b + " the camera. Please try to manually calibrate it.")
            return False,None,None
        rate.sleep()

grabbed, trans, rot = get_tf('/base','/left_hand_camera')
print(grabbed)
print(trans)
print(rot)
#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
#import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('baxter_tf_listener')

    listener = tf.TransformListener()


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base','/left_hand_camera',  rospy.Time(0))
            print(trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Error")
            continue

        """angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)"""

        rate.sleep()
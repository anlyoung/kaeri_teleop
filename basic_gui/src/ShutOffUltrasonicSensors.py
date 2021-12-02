#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16

import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--data', type=int,
                    help='what to set sonar sensor', default = 0)
args = parser.parse_args()

data_to_set = args.data

currentState = None
def subscriber_callback(data):
    global currentState
    currentState = data.data
    #print("Current State: ",currentState)
    
def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.


    pub = rospy.Publisher('/robot/sonar/head_sonar/set_sonars_enabled', UInt16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("/robot/sonar/head_sonar/sonars_enabled", UInt16, subscriber_callback)
    rate = rospy.Rate(100) # 10hz
    while currentState != data_to_set :
        pub.publish(data_to_set)
        #print("Publish Data:" + str(data_to_set))
        rate.sleep()

if __name__ == '__main__':
    main()
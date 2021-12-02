#!/usr/bin/env python

import rospy

from baxter_core_msgs.msg import EndpointState

import csv

pos = []

def callback(msg):
    print (msg.pose.position)
    pos.append([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])



rospy.init_node('SaveEndPoint',anonymous=True)


rospy.Subscriber('/robot/limb/right/endpoint_state',EndpointState,callback,queue_size = 1)

rospy.spin()

with open('BaxterEndpointPosition.csv', mode='w') as csv_file:
    csv_writer = csv.writer(csv_file,delimiter=',')

    for x in pos:
        csv_writer.writerow(x)


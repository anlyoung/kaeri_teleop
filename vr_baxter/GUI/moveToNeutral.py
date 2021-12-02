#!/usr/bin/env python


import rospy
import rospkg

import baxter_interface

import struct
import sys
import csv
from time import sleep
from threading import Lock,Thread

limb_joints_right = {'right_s0': -1.6885293522648692, 'right_s1': -1.3115535736420287, 'right_w0': -0.19059711289476267, 'right_w1': 0.8498253564885192, 'right_w2': -0.8298836062460089,   'right_e0': 0.9150195399736493, 'right_e1': 2.0014614329934934}
limb_joints_left = {'left_w0': 0.6795534890332383, 'left_w1': 1.0300680990650553, 'left_w2': -0.3401602397135905,   'left_e0': -1.1455001533534328, 'left_e1': 1.8733740372050616, 'left_s0': 1.29774774655106, 'left_s1': -0.9303593478525034, }



def move_right():
    #Moving Baxter Arm
    l = baxter_interface.Limb("right")

    print("Moving Right Arm ...")
    l.move_to_joint_positions(limb_joints_right, timeout = 10,threshold=0.05)
    print("Done Moving Right Arm")


def move_left():
    #Moving Baxter Arm
    l = baxter_interface.Limb("left")

    print("Moving Left Arm ...")
    l.move_to_joint_positions(limb_joints_left, timeout = 10,threshold=0.05)
    print("Done Moving Left Arm")

rospy.init_node('MoveToNewutral',anonymous=True)
#start Thread
lock = Lock()
processThread = Thread(target=move_right)
Thread.daemon = True
processThread.start()

move_left()

while (processThread.is_alive()):
    pass

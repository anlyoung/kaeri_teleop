#!/usr/bin/env python

import rospy
import rospkg

import baxter_interface

import argparse

from update_joint import cal_position
from update_joint import FILE_NAME
from update_joint import PACKAGE

import struct
import sys
import csv
from time import sleep
from threading import Lock,Thread

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.msg import EndpointState
"""from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)"""
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

limb = "right"
rospack = rospkg.RosPack()
path = rospack.get_path('pbdlib_manager')
file = path + '/Test/Unity/optimalData1.txt'
def read_pose_file(file):

    cal_pos = cal_position(FILE_NAME,PACKAGE).read_csv()
    PosList = []
    with open(file, 'r') as csvFile:
        reader = csv.reader(csvFile)
        for row in reader:
            list_test = []
            for item in row:
                list_test.append(float(item))
            PosList.append(list_test)
    csvFile.close()

    my_array = PoseArray()
    my_array.header.frame_id = "MotionPrimatives"

    for n in range(len(PosList[0])):
        my_array.poses.append(Pose(
            position=Point(
                x=PosList[0][n],
                y=PosList[1][n],
                z=PosList[2][n],
            ),
            orientation=Quaternion(
                #x=0.0319755392568,
                #y=0.708971762152,
                #z=-0.0470461373655,
                #w=0.70293902034,
                x = cal_pos.get_right_quaternion()['x'],
                y = cal_pos.get_right_quaternion()['y'],
                z = cal_pos.get_right_quaternion()['z'],
                w = cal_pos.get_right_quaternion()['w'],
            ),

        )) 
    return my_array

rospy.init_node('UnityPos',anonymous=True)

#wait_to_finish waits until the arm gets to the desired position before continueing
def ik_test(limb,pos,rot,wait_to_finish = False):
    #rospy.init_node("rsdk_ik_service_client")
    cal_pos = cal_position(FILE_NAME,PACKAGE).read_csv()
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=pos.x,
                    y=pos.y,
                    z=pos.z,
                ),
                orientation=Quaternion(
                    #x = rot.x,
                    #y = rot.y,
                    #z = rot.z,
                    #w = rot.w,
                    x = cal_pos.get_left_quaternion()['x'],
                    y = cal_pos.get_left_quaternion()['y'],
                    z = cal_pos.get_left_quaternion()['z'],
                    w = cal_pos.get_left_quaternion()['w'],

                    #x= -0.281380921476,
                    #y= 0.100179126807,
                    #z= -0.353019092049,
                    #w= 0.88666027329,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= pos.x,
                    y= pos.y,
                    z= pos.z,
                ),
                orientation=Quaternion(
                    x = cal_pos.get_right_quaternion()['x'],
                    y = cal_pos.get_right_quaternion()['y'],
                    z = cal_pos.get_right_quaternion()['z'],
                    w = cal_pos.get_right_quaternion()['w'],

                    #x= 0.163248285159,
                    #y= 0.0756043271539,
                    #z= 0.299289706451,
                    #w= 0.937048373736,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        """print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))"""
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        # Format solution into Limb API-compatible dictionary
        #print "\nIK Joint Solution:\n", limb_joints
        #print "------------------"
        #print "Response Message:\n", resp

        print("SUCCESS - Moving Arm")
        #Moving Baxter Arm
        l = baxter_interface.Limb(limb)
        if (wait_to_finish): l.move_to_joint_positions(limb_joints, timeout = 10,threshold=0.05)
        else:l.set_joint_positions(limb_joints)

    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        #print(pos.x,pos.y,pos.z)
        #print(poses[limb])

    return 0


def callback(msg):
	ik_test('left',msg.pose.position,msg.pose.orientation)




Thread_stopped = False
def control_arm():
    with lock: pose_to_find_local = pose_to_find
    while (not rospy.is_shutdown()) and (not Thread_stopped):
        #print("ik_test")
        ik_test(limb,pose_to_find.position,None)

endpoint = None
def get_endpoint(data):
    global endpoint
    endpoint = data

cylinder_pose = None
def get_cylinder(data):
    global cylinder_pose
    cylinder_pose = data

control_speed = 25
lock = Lock()
pose_to_find = None
offset = [0,0,0]
offset = [0.59446189702,-0.086670205326,0.555096093249]#x,y,z

start_pose = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-x", help="where the robot should start",
                        type=float,default = 0,required=False)
    parser.add_argument("-y", help="where the robot should start",
                        type=float,default = 0,required=False)
    parser.add_argument("-z", help="where the robot should start",
                        type=float,default = 0,required=False)
    parser.add_argument("-d", "--default", action="store_true")
    parser.add_argument("-p", "--position", action="store_true")
    parser.add_argument("-c", "--cylinder", action="store_true")
    args = parser.parse_args()  
    offset = [args.x,args.y,args.z]

    global pose_to_find,Thread_stopped,endpoint


    
    my_pose_array = read_pose_file(file)
    if ( args.default):
        pose_to_find = my_pose_array.poses[0]
    elif ( args.position):
        rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_endpoint)
        while endpoint== None:pass
        begin = endpoint
        offset[0] = begin.pose.position.x
        offset[1] = begin.pose.position.y
        offset[2] = begin.pose.position.z
        
        pose_to_find = begin.pose
    elif ( args.cylinder):
        rospy.Subscriber("/VR_Cylinder_Pos", Pose, get_cylinder)
        while cylinder_pose== None:pass
        cylinder_pose_begin = cylinder_pose

        pose_to_find = my_pose_array.poses[0]

        offset[0] += cylinder_pose_begin.position.z
        offset[1] += -1*cylinder_pose_begin.position.x
        offset[2] += .15 #arbitrary #cylinder_pose_begin.position.y
        
        #where it starts
        pose_to_find.position.x += offset[0] 
        pose_to_find.position.y += offset[1] 
        pose_to_find.position.z += offset[2]
        print(offset)


        offset[0] +=  my_pose_array.poses[0].position.x
        offset[1] +=  my_pose_array.poses[0].position.y
        offset[2] +=  my_pose_array.poses[0].position.z

        print(pose_to_find)
    else:
        pose_to_find = Pose()
        pose_to_find.position.x = offset[0]
        pose_to_find.position.y = offset[1]
        pose_to_find.position.z = offset[2]

    start_pose = my_pose_array.poses[0]

    #print(my_pose_array.poses[0])
    #print(pose_to_find)

    rate = rospy.Rate(control_speed)

    #go to start
    ik_test(limb,pose_to_find.position,None,wait_to_finish =True)#waits till gets to the start

    #start Thread
    lock = Lock()
    processThread = Thread(target=control_arm)
    Thread_stopped = False
    Thread.daemon = True
    processThread.start()

    for n in range(1,len(my_pose_array.poses)):
        with lock: 
            pose_to_find = my_pose_array.poses[n]
            if (not args.default):
                pose_to_find.position.x += offset[0] - start_pose.position.x
                pose_to_find.position.y += offset[1] - start_pose.position.y
                pose_to_find.position.z += offset[2] - start_pose.position.z
        #print("updated the pose")
        rate.sleep()
    Thread_stopped = True
    sleep(.03)

if __name__=='__main__':
	main()

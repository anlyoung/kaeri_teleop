#!/usr/bin/env python

import rospy

import baxter_interface

from update_joint import cal_position
from update_joint import FILE_NAME
from update_joint import PACKAGE

import struct
import sys

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

rospy.init_node('UnityPos',anonymous=True)

def ik_test(limb,pos,rot):
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
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        l = baxter_interface.Limb(limb)
        l.set_joint_positions(limb_joints)
        #print "\nIK Joint Solution:\n", limb_joints
        #print "------------------"
        #print "Response Message:\n", resp
        
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        #print(pos)

    return 0


def callback(msg):
	ik_test('left',msg.pose.position,msg.pose.orientation)

def main():
	rospy.Subscriber('/Unity/LeftControllerPosition',PoseStamped,callback,queue_size = 1)

	rospy.spin()

if __name__=='__main__':
	main()

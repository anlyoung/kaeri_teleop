#!/usr/bin/env python

import rospy

import baxter_interface

import struct
import sys

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from sensor_msgs.msg import (
    JointState
)

from intera_core_msgs.msg import (
    JointCommand
)

from std_msgs.msg import Header
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

rospy.init_node('UnityPos',anonymous=True)

pub = rospy.Publisher('/sawyer/CalculatedJointPosition', JointState, queue_size=1)
pub1 = rospy.Publisher('/robot/limb/right/joint_command', JointCommand,queue_size=1)


def ik_sawyer(limb, use_advanced_options,pos,rot):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=pos.x/1.5,
                    y=pos.y/1.5,
                    z=pos.z/1.5,
                ),
                orientation=Quaternion(
                    #x=rot.x,
                    #y=rot.y,
                    #z=rot.z,
                    #w=rot.w,
                    x = .86,
                    y = -.505,
                    z = -.006,
                    w=.0306,
                ),
            ),
        ),
    }
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    if (use_advanced_options):
        # Optional Advanced IK parameters
        rospy.loginfo("Running Advanced IK Service Client example.")
        # The joint seed is where the IK position solver starts its optimization
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is 
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(True)
        # The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [0.1, -0.3, 0.5]
        ikreq.nullspace_goal.append(goal)
        # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
        # If empty, the default gain of 0.4 will be used
        ikreq.nullspace_gain.append(0.4)
    else:
        rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", resp)

        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        js.position = [0,0,0,0,0,0,0]
        js.velocity = []
        js.effort = []

        jc = JointCommand()
        jc.mode = 1
        jc.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        jc.position = [0,0,0,0,0,0,0]



        for x in range (0,7):
            js.position[x] = resp.joints[0].position[x]
            jc.position[x] = resp.joints[0].position[x]

        pub.publish(js)

        pub1.publish(jc)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return False

    return True

def callback(msg):
	ik_sawyer("right",False,msg.position,msg.orientation)

def main():
	rospy.Subscriber('/Phantom_joint_states',Pose,callback,queue_size = 1)

	rospy.spin()

if __name__=='__main__':
	main()

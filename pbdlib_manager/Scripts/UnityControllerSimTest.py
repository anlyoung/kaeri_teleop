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
from sensor_msgs.msg import JointState
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class robot_position(object):
    def __init__(self):
        #untucked position
        self.last_position ={'left_w0': 0.6680486331240977, 
                        'left_w1': 1.0281506230801987, 'left_w2': -0.49777676566881673, 
                        'right_s0': 0.08590292412158317, 'right_s1': -1.0097428536255737, 
                        'right_w0': -0.6653641667452982, 'right_w1': 1.022014699928657, 
                        'head_pan': -0.10929613113683573, 'right_w2': 0.4962427848809313, 
                        'head_nod': 0.0, 'torso_t0': -12.565987119160338, 
                        'left_e0': -1.1846166634445108, 'left_e1': 1.9381847254932203, 
                        'left_s0': -0.08130098175792692, 'left_s1': -1.002456444883118, 
                        'right_e0': 1.1562380188686305, 'right_e1': 1.9462381246296188}
        self.msg = JointState()
        self.msg.header = Header()
        self.msg.velocity = []
        self.msg.effort = []

        self.seedRight = JointState()
        self.seedRight.header = Header()
        self.seedRight.velocity = []
        self.seedRight.effort = []
        self.seedRight.name = ['right_s0', 'right_s1', 'right_w0', 'right_w1',
                               'right_w2', 'right_e0', 'right_e1']

        self.seedLeft = JointState()
        self.seedLeft.header = Header()
        self.seedLeft.velocity = []
        self.seedLeft.effort = []
        self.seedLeft.name =  ['left_s0', 'left_s1', 'left_w0', 'left_w1',
                               'left_w2', 'left_e0', 'left_e1']
    def get_last_joint_states(self):
        return last_position
    def set_joint_states(self,joint_states):
        for joint in joint_states:
            self.last_position[joint] = joint_states[joint]
    def get_message_joint_states(self):
        self.msg.name = self.last_position.keys()
        self.msg.position = self.last_position.values()
        return self.msg
    def get_seed(self,limb):
        if limb == "right":
            self.seedRight.position = [self.last_position[name] for name in self.seedRight.name ]
            return self.seedRight
        else:
            self.seedleft.position = [self.last_position[name] for name in self.seedLeft.name ]
            return self.seedLeft

rospy.init_node('UnityPos',anonymous=True)

def ik_test(limb,pos,rot):

    cal_pos = cal_position(FILE_NAME,PACKAGE).read_csv()

    #rospy.init_node("rsdk_ik_service_client")
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
                    #x = 0.916731925309,
                    #y = -0.319535935555,
                    #z = -0.0855042032591,
                    #w = -0.224027663992,
                    x = cal_pos.get_left_quaternion()['x'],
                    y = cal_pos.get_left_quaternion()['y'],
                    z = cal_pos.get_left_quaternion()['z'],
                    w = cal_pos.get_left_quaternion()['w'],

                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=pos.x,
                    y=pos.y,
                    z= pos.z,
                    #x=0.656982770038,
                    #y=-0.852598021641,
                    #z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x = cal_pos.get_right_quaternion()['x'],
                    y = cal_pos.get_right_quaternion()['y'],
                    z = cal_pos.get_right_quaternion()['z'],
                    w = cal_pos.get_right_quaternion()['w'],
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    ikreq.seed_mode = ikreq.SEED_USER
    seed = current_robot_position.get_seed(limb)
    ikreq.seed_angles.append(seed)
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
        #print "\nIK Joint Solution:\n", limb_joints
        #print "------------------"
        #print "Response Message:\n", resp
        return limb_joints
        
    else:
        #print("INVALID POSE - No Valid Joint Solution Found.")
        return None

current_robot_position = robot_position()

def callbackleft(msg):
    result = ik_test('left',msg.pose.position,msg.pose.orientation)
    if result is not None:
        current_robot_position.set_joint_states(result)
        #print("YesL")


def callbackright(msg):
    result = ik_test('right',msg.pose.position,msg.pose.orientation)
    if result is not None:
        current_robot_position.set_joint_states(result)

def main():
    #rospy.Subscriber('/MotionPrimatives/HandPoseStamped',PoseStamped,callbackleft,queue_size = 1)
    rospy.Subscriber('/MotionPrimitives/HandPoseStamped',PoseStamped,callbackright,queue_size = 1)

    pub = rospy.Publisher('/MotionPrimitives/BaxterPosition', JointState, queue_size=10)

    rate = rospy.Rate(90) # 10hz
    while not rospy.is_shutdown():
        message = current_robot_position.get_message_joint_states()
        pub.publish(message)
        rate.sleep()

if __name__=='__main__':
	main()

#!/usr/bin/env python
# license removed for brevity
import rospy
from  baxter_core_msgs.msg import JointCommand
from  sensor_msgs.msg import JointState

positions = [0,0,0,0,0,0,0]

def callback(data):
    positions = data.position[2:9]
    positions = list(positions)
    positions[0] *= -1
    positions[2] *= -1
    positions[4] *= -1
    positions[6] *= -1
    pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size=1)
    jc = JointCommand()
    jc.mode = 1
    jc.command = positions
    jc.names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    #rospy.loginfo(jc)
    pub.publish(jc)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.[
    rospy.init_node('listener_left_joint', anonymous=True)

    rospy.Subscriber("robot/joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

"""
def talker():
    pub = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size=10)
    rospy.init_node('talker_joint_command', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        jc = JointCommand()
        jc.mode = 1
        jc.command = positions.copy()
        jc.names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
        rospy.loginfo(jc)
        pub.publish(jc)
        rate.sleep()
"""
if __name__ == '__main__':
    listener()


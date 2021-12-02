#!/usr/bin/env python
import rospy
from baxter_core_msgs.msg import EndpointState
import rospkg
import os.path
from os import path
from os import makedirs
import csv



FILE_NAME = "joint_end_states_cal.txt"
PACKAGE = "vr_baxter"

class reset_position:
    def __init__(self,file_name,package):

        self.file_name= file_name
        self.package = package

        self.left_arm_finished = False
        self.right_arm_finished = False

        self.right_quat = ()#x,y,z,w
        self.left_quat = ()#x,y,z,w


    def callback_left_arm(self,data):
        if self.left_arm_finished:return
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.orientation)


        ori = data.pose.orientation
        self.left_quat = (ori.x,ori.y,ori.z,ori.w)

        self.left_arm_finished = True


    def callback_right_arm(self,data):
        if self.right_arm_finished:return
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.orientation)


        ori = data.pose.orientation
        self.right_quat = (ori.x,ori.y,ori.z,ori.w)

        self.right_arm_finished = True
    
    def write_to_file(self):

        # Opening and Closing a file "MyFile.txt" 
        # for object name file1. 
        r = rospkg.RosPack()
        folder = r.get_path(self.package)+"/calibrate"
        if (not path.exists(folder)):
            makedirs(folder)


        full_file_name = folder+ "/"+self.file_name
        print(full_file_name)

        file1 = open(full_file_name,"w") 
        file1.write("Right Arm\n") 
        file1.write("%s,%s,%s,%s\n"%self.right_quat) 
        file1.write("Left Arm\n") 
        file1.write("%s,%s,%s,%s\n"%self.left_quat) 
        file1.close() 
        



    def start(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.callback_left_arm)
        rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.callback_right_arm)

        # spin() simply keeps python from exiting until this node is stopped
        while not(self.left_arm_finished and self.right_arm_finished):pass
        self.write_to_file()
        print("Calibration Successful")


class cal_position:
    def __init__(self,file_name,package):

        self.file_name= file_name
        self.package = package
        r = rospkg.RosPack()
        folder = r.get_path(self.package)+"/calibrate"
        self.full_file_name = folder+ "/"+self.file_name


        self.right_quat = {}
        self.left_quat = {}

    def read_csv(self):
        with open(self.full_file_name) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if(line_count == 1):
                    self.right_quat = {
                        "x":float(row[0]),
                        "y":float(row[1]),
                        "z":float(row[2]),
                        "w":float(row[3])
                    }
                if(line_count == 3):
                    self.left_quat = {
                        "x":float(row[0]),
                        "y":float(row[1]),
                        "z":float(row[2]),
                        "w":float(row[3])
                    }
                line_count += 1
        return self
    def get_right_quaternion(self):return self.right_quat
    def get_left_quaternion(self):return self.left_quat




if __name__ == '__main__':
    reset_position(FILE_NAME,PACKAGE).start()
    cal_pos = cal_position(FILE_NAME,PACKAGE).read_csv()

    print(cal_pos.get_left_quaternion())
    print(cal_pos.get_right_quaternion())

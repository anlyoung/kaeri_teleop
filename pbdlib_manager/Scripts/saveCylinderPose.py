#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import rospkg
import os.path
from os import path
from os import makedirs
import csv
import time

#!/usr/bin/env python
import rospy
from baxter_core_msgs.msg import EndpointState
import rospkg
import os.path
from os import path
from os import makedirs
import csv



FILE_NAME = "cylinder_pos.txt"
TEST = "Unity"
PACKAGE = "pbdlib_manager"

class store_cyl_pos:
    def __init__(self,file_name,package,test,timeout = 10000):

        self.file_name= file_name
        self.package = package
        self.test = test
        self.timeout = timeout

        self.finished = False

        self.cyl_pose = ()#x,y,z,w


    def callback(self,data):
        if self.finished:return
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)


        self.cyl_pose = (data.position.x,data.position.y,data.position.z)

        self.finished = True


    def write_to_file(self):

        # Opening and Closing a file "MyFile.txt" 
        # for object name file1. 
        r = rospkg.RosPack()
        folder = r.get_path(self.package)+"/Test/"+self.test
        if (not path.exists(folder)):
            makedirs(folder)


        full_file_name = folder+ "/"+self.file_name
        print(full_file_name)

        file1 = open(full_file_name,"w") 
        file1.write("Cylinder Position\n") 
        file1.write("%s,%s,%s\n"%self.cyl_pose) 
        



    def start(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/VR_Cylinder_Pos", Pose, self.callback)
        
        start_millis = int(round(time.time() * 1000))
        curr_millis = start_millis
        # spin() simply keeps python from exiting until this node is stopped
        while (not(self.finished) and (curr_millis-start_millis < self.timeout)):
            curr_millis = int(round(time.time() * 1000))

        if self.finished:
            self.write_to_file()
            print("Calibration Successful")
        else:
            print("No data to write")


class pull_stored_Cyl_position:
    def __init__(self,file_name,package,test):

        self.file_name= file_name
        self.package = package
        self.test =test
        r = rospkg.RosPack()
        folder = r.get_path(self.package)+"/Test/"+self.test
        self.full_file_name = folder+ "/"+self.file_name


        self.cyl_pos = {}

    def get_cyl_pos(self):
        with open(self.full_file_name) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if(line_count == 1):
                    self.cyl_pos = {
                        "x":float(row[0]),
                        "y":float(row[1]),
                        "z":float(row[2])
                    }
                    break
                line_count += 1
        return self.cyl_pos




if __name__ == '__main__':
    store_cyl_pos(FILE_NAME,PACKAGE,TEST).start()

    print(pull_stored_Cyl_position(FILE_NAME,PACKAGE,TEST).get_cyl_pos())

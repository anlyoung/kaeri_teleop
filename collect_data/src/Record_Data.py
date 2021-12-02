#!/usr/bin/env python
import csv
import time
import os, shutil
import rospy
from vr_interface_msgs.msg import PoseNameArray
from std_msgs.msg import Int32
names = ["Tracker", "RightHand", "ThumbFinger", "IndexFinger", "MiddleFinger", "RingFinger", "PinkyFinger"]

folder = '/home/user/catkin_ws/src/collect_data/Data/Trial1'
for the_file in os.listdir(folder):
    file_path = os.path.join(folder, the_file)
    try:
        if os.path.isfile(file_path):
            os.unlink(file_path)
        elif os.path.isdir(file_path): shutil.rmtree(file_path)
    except Exception as e:
        print(e)


for item in names:
    location = os.path.join(folder,item)
    try:
        os.makedirs(location)
        print("Created folder" + location)
    except OSError:
        pass

def save_file(file_name,pos):
    with open(file_name, 'wb') as myfile:
        wr = csv.writer(myfile)
        for item in pos:
            wr.writerow(item)

def save_trial():
    global current_trial,trial_number
    for name in current_trial.keys():
        text_file = os.path.join(folder, name,str(trial_number)+".txt")
        save_file(text_file,current_trial[name])
    


Trial_template = {}
for item in names:
    Trial_template[item] = [[],[],[],[]]
current_trial = Trial_template.copy()
#new_item = os.path.join(folder, "0.csv")
#save_file(new_item,[[1,2,3],[4,5,6],[7,8,9]])
trial_number = 0
recording = False
print("Moving to Trial 0")

def getPoseNameDict(msg):
    return dict(zip(msg.names,msg.poses))

def callback_data(data):
    global trial_number,start_time,should_record_start
    current_time = time.time()

    poses = getPoseNameDict(data)

    if (recording):
        if should_record_start:
            start_time = current_time
            should_record_start =False
        for name in names: 
            current_trial[name][0].append(poses[name].position.x)
            current_trial[name][1].append(poses[name].position.y)
            current_trial[name][2].append(poses[name].position.z)
            current_trial[name][3].append(current_time - start_time)

    #rospy.loginfo(poses)
    #rospy.loginfo(poses["camera_link"].position)
    #rospy.loginfo(poses["Headset"].position)

start_time = 0
should_record_start = False
def callback_int(data):
    global trial_number,recording,start_time,should_record_start
    if (data.data == 2):
        recording = True
    else:
        recording = False
    if (data.data == 3):
        save_trial()
        trial_number+=1
        current_trial = Trial_template.copy()
        print("Moving to Trial " + str(trial_number) + "      " )
    if (data.data == 1):
        should_record_start = True
        print("Recording...")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/HTCVive/RightHandGlove", PoseNameArray, callback_data)
    rospy.Subscriber("/HTCVive/ButtonPressed", Int32, callback_int)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
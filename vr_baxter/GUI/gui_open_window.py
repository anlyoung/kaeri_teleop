#!/usr/bin/env python

# imports so you can do lots of things!
import Tkinter as tk
import os
import signal
import subprocess
import roslaunch
from LaunchRunClass import *

#subprocesses
"""global main_subprocess
global right_controller_subprocess
global left_controller_subprocess 
global Eucledian_Extraction_subprocess"""


right_controller_subprocess = None 
left_controller_subprocess = None 
Eucledian_Extraction_subprocess = None 

import time
import sys
path = sys.path[0]

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

import atexit

#kill all programs on exit
def exit_handler():
	kill_all()

atexit.register(exit_handler)



# functions that you write for your buttons!!
list_of_classes = []
def kill_all():
	print("Stoping all Processes...")
	mainLaunchKinfu.stop()
	for class_name in list_of_classes:
		class_name.stop()
	print("All Processes Stopped")

moveToNeutral= CustomNodeRun("vr_baxter", "moveToNeutral.py")
list_of_classes.append(moveToNeutral)
def r_move_neutral():
	moveToNeutral.stop()
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun vr_baxter moveToNeutral.py'"
		subprocess.call(command,shell = True)
	else:
		moveToNeutral.start()

calibrate_end_pose = CustomNodeRun("vr_baxter", "update_joint.py")
list_of_classes.append(calibrate_end_pose)
def r_calibrate_end_pose():
	calibrate_end_pose.stop()
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun vr_baxter update_joint.py'"
		subprocess.call(command,shell = True)
	else:
		calibrate_end_pose.start()

mainLaunchKinfu = CustomLaunchRun("kinfu", "kinect_single.launch")
mainLaunchFileserver= CustomLaunchRun("file_server", "publish_description_baxter.launch")
list_of_classes.append(mainLaunchKinfu)
list_of_classes.append(mainLaunchFileserver)
def s_demo_callback():
	kill_all()
	print "we're gonna start the demo now!!"
	s_kinfu()
	s_file_server()
	#r_PC_callback()
	r_LC_callback()
	r_RC_callback()

def s_kinfu():
	mainLaunchKinfu.stop()
	print "Starting Kinfu!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'roslaunch kinfu kinect_single.launch'"
		subprocess.call(command,shell = True)
	else:
		mainLaunchKinfu.start()	
def s_file_server():
	mainLaunchFileserver.stop()
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'roslaunch file_server publish_description_baxter.launch'"
		subprocess.call(command,shell = True)
	else:
		mainLaunchFileserver.start()	

PCLaunch = CustomNodeRun("vr_baxter", "EuclideanExtraction")
list_of_classes.append(PCLaunch)
def r_PC_callback():
	PCLaunch.stop()
	print "restarting the point cloud since things got messy!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun vr_baxter EuclideanExtraction'"
		subprocess.call(command,shell = True)
	else:
		PCLaunch.start()

LeftCLaunch = CustomNodeRun("vr_baxter", "UnityController.py")
list_of_classes.append(LeftCLaunch)
def r_LC_callback():
	LeftCLaunch.stop()
	print "sometimes baxter's left arm doesn't work. so we're trying again!!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun vr_baxter UnityController.py'"
		subprocess.call(command,shell = True)
	else:
		LeftCLaunch.start()

RightCLaunch = CustomNodeRun("vr_baxter", "UnityControllerRight.py")
list_of_classes.append(RightCLaunch)
def r_RC_callback():
	RightCLaunch.stop()
	print "dang, baxter's right arm is not working as expected. take 2!!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun vr_baxter UnityControllerRight.py'"
		subprocess.call(command,shell = True)
	else:
		RightCLaunch.start()
	
TuckLaunch = CustomNodeRun("baxter_tools", "tuck_arms.py","-t")
list_of_classes.append(TuckLaunch)
def Tucking_callback():
	print "Tucking Baxters Arms. Goodbye!"
	kill_all()
	TuckLaunch.start()

UntuckLaunch = CustomNodeRun("baxter_tools", "tuck_arms.py","-u")
list_of_classes.append(TuckLaunch)
def Untucking_callback():
	print "Untucking Baxter's arms. Soon he will be ready to go!!"
	kill_all()
	UntuckLaunch.start()

# main/root window stuff
WINDOW_SIZE = "650x120"

root = tk.Tk()
root.title("VR Baxter Demo")
root.geometry(WINDOW_SIZE)


# frame stuff (helps makes things neater in the main/root window
frame = tk.Frame(root)
frame.pack()

middleframe = tk.Frame(root)
middleframe.pack( )

thirdframe = tk.Frame(root)
thirdframe.pack( )

bottomframe = tk.Frame(root)
bottomframe.pack( )


# buttons stuff that have functions you define!!
startbutton = tk.Button(frame, text="Start Demo", fg="green", command= s_demo_callback)
startbutton.pack(side = tk.LEFT)

# buttons stuff that have functions you define!!
killbutton = tk.Button(frame, text="Kill Demo", fg="red", command=kill_all)
killbutton.pack(side = tk.LEFT)

UseGnomeTerminal = tk.IntVar()
c = tk.Checkbutton(frame, text="Open New Terminals", variable=UseGnomeTerminal)
c.pack()

Untuckbutton = tk.Button(middleframe, text="Untuck Baxter's Arm", fg = "black", command=Untucking_callback)
Untuckbutton.pack(side=tk.LEFT)

Tuckbutton = tk.Button(middleframe, text="Tuck Baxter's Arm", fg = "black", command=Tucking_callback)
Tuckbutton.pack(side=tk.LEFT)

Tuckbutton = tk.Button(middleframe, text="Move To Neutral", fg = "black", command=r_move_neutral)
Tuckbutton.pack(side=tk.LEFT)

CalibrateEndbutton = tk.Button(middleframe, text="Calibrate End Pose", fg = "black", command=r_calibrate_end_pose)
CalibrateEndbutton.pack(side=tk.LEFT)


Extractbutton = tk.Button(thirdframe, text="Start Euclidean Extraction", fg = "black", command=r_PC_callback)
Extractbutton.pack(side=tk.LEFT)

LCbutton=tk.Button(thirdframe, text="Start L-Controller", fg="black", command=r_LC_callback)
LCbutton.pack(side=tk.LEFT)

RCbutton=tk.Button(thirdframe, text="Start R-Controller", fg="black", command=r_RC_callback)
RCbutton.pack(side=tk.LEFT)

Fileserverbutton = tk.Button(bottomframe, text="Start File Server", fg = "black", command=s_file_server)
Fileserverbutton.pack(side=tk.LEFT)

Kinfubutton = tk.Button(bottomframe, text="Start Kinfu", fg = "black", command=s_kinfu)
Kinfubutton.pack(side=tk.LEFT)

if __name__ == '__main__':
	root.mainloop()

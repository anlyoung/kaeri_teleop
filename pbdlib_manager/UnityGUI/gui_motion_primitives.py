#!/usr/bin/env python

# main/root window stuff
WINDOW_SIZE = "550x140"

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
	for class_name in list_of_classes:
		class_name.stop()
	print("All Processes Stopped")

mainLaunchFileserver= CustomLaunchRun("file_server", "publish_description_baxter.launch")
list_of_classes.append(mainLaunchFileserver)
def s_file_server():
	mainLaunchFileserver.stop()
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'roslaunch file_server publish_description_baxter.launch'"
		subprocess.call(command,shell = True)
	else:
		mainLaunchFileserver.start()	


simNode= CustomNodeRun("pbdlib_manager", "UnityControllerSimTest.py")
list_of_classes.append(simNode)
print 3.2
def r_sim_callback():
	simNode.stop()
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun pbdlib_manager UnityControllerSimTest.py'"
		subprocess.call(command,shell = True)
	else:
		simNode.start()


def r_MP_callback():
	print "Starting the simulation IK!"
	command = "gnome-terminal -x bash -c 'roslaunch pbdlib_manager pbdlib_baxter_demo.launch'"
	subprocess.call(command,shell = True)


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


show_data_node= CustomNodeRun("pbdlib_manager", "plotMotionPrimitives.py")
list_of_classes.append(show_data_node)
def r_show_data_callback():
	show_data_node.stop()
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun pbdlib_manager plotMotionPrimitives.py'"
		subprocess.call(command,shell = True)
	else:
		show_data_node.start()


mainLaunchKinfu = CustomLaunchRun("kinfu", "kinect_single.launch")
list_of_classes.append(mainLaunchKinfu)
def s_kinfu():
	mainLaunchKinfu.stop()
	print "Starting Kinfu!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'roslaunch kinfu kinect_single.launch'"
		subprocess.call(command,shell = True)
	else:
		mainLaunchKinfu.start()	


moveToNeutral= CustomNodeRun("pbdlib_manager", "moveToNeutral.py")
list_of_classes.append(moveToNeutral)
def r_move_neutral():
	moveToNeutral.stop()
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun pbdlib_manager moveToNeutral.py'"
		subprocess.call(command,shell = True)
	else:
		moveToNeutral.start()


run_model_node= CustomNodeRun("pbdlib_manager", "RunUnityModel.py","-d")
list_of_classes.append(run_model_node)
def r_model_callback():
	"""global run_model_node
	run_model_node.stop()
	if UsePositionPath.get():
		run_model_node= CustomNodeRun("pbdlib_manager", "RunUnityModel.py",'-p')
	else:
		run_model_node= CustomNodeRun("pbdlib_manager", "RunUnityModel.py",'-d')

	#run_model_node= CustomNodeRun("pbdlib_manager", "RunUnityModel.py")
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun pbdlib_manager RunUnityModel.py -d'"
		subprocess.call(command,shell = True)
	else:
		run_model_node.start()"""
	if UsePositionPath.get():
		command = "gnome-terminal -x bash -c 'rosrun pbdlib_manager RunUnityModel.py -p'"
	elif UseCylinderPath.get():
		command = "gnome-terminal -x bash -c 'rosrun pbdlib_manager RunUnityModel.py -c'"
	else:
		command = "gnome-terminal -x bash -c 'rosrun pbdlib_manager RunUnityModel.py -d'"
	subprocess.call(command,shell = True)


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

calibrate_end_pose = CustomNodeRun("pbdlib_manager", "update_joint.py")
list_of_classes.append(calibrate_end_pose)
def r_calibrate_end_pose():
	calibrate_end_pose.stop()
	print "Starting File Server!"
	if UseGnomeTerminal.get():
		command = "gnome-terminal -x bash -c 'rosrun pbdlib_manager update_joint.py'"
		subprocess.call(command,shell = True)
	else:
		calibrate_end_pose.start()
		
root = tk.Tk()
root.title("Motion Primitives Baxter Demo")
root.geometry(WINDOW_SIZE)


# frame stuff (helps makes things neater in the main/root window
frame = tk.Frame(root)
frame.pack()

middleframe = tk.Frame(root)
middleframe.pack( )

thirdframe = tk.Frame(root)
thirdframe.pack( )

forthframe = tk.Frame(root)
forthframe.pack( )


bottomframe = tk.Frame(root)
bottomframe.pack( )


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

CalibrateEndbutton = tk.Button(middleframe, text="Calibrate End Pose", fg = "black", command=r_calibrate_end_pose)
CalibrateEndbutton.pack(side=tk.LEFT)

Extractbutton = tk.Button(thirdframe, text="Start File Server", fg = "black", command=s_file_server)
Extractbutton.pack(side=tk.LEFT)

LCbutton=tk.Button(thirdframe, text="Start Simulation IK", fg="black", command=r_sim_callback)
LCbutton.pack(side=tk.LEFT)

RCbutton=tk.Button(thirdframe, text="Start Motion Primitives", fg="black", command=r_MP_callback)
RCbutton.pack(side=tk.LEFT)

Kinfubutton = tk.Button(forthframe, text="Start Kinfu", fg = "black", command=s_kinfu)
Kinfubutton.pack(side=tk.LEFT)

Extractbutton = tk.Button(forthframe, text="Move To Neutral", fg = "black", command=r_move_neutral)
Extractbutton.pack(side=tk.LEFT)

Extractbutton = tk.Button(forthframe, text="Start Euclidean Extraction", fg = "black", command=r_PC_callback)
Extractbutton.pack(side=tk.LEFT)

Fileserverbutton = tk.Button(bottomframe, text="Show data", fg = "black", command=r_show_data_callback)
Fileserverbutton.pack(side=tk.LEFT)

Kinfubutton = tk.Button(bottomframe, text="Run Model", fg = "black", command=r_model_callback)
Kinfubutton.pack(side=tk.LEFT)

UsePositionPath = tk.IntVar()
d = tk.Checkbutton(bottomframe, text="Run at Current Position", variable=UsePositionPath)
d.pack(side=tk.LEFT)

UseCylinderPath = tk.IntVar()
d = tk.Checkbutton(bottomframe, text="Run near cylinder", variable=UseCylinderPath)
d.pack(side=tk.LEFT)


if __name__ == '__main__':
	root.mainloop()

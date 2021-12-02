#!/usr/bin/env python

# imports so you can do lots of things!
import Tkinter as tk
import os
import signal
import subprocess


import time
import sys
path = sys.path[0]

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()



import atexit

def kill_all():
	print 'The Demo is ending'
	kill_mirror()

atexit.register(kill_all)

mirror_subprocess = None
def kill_mirror():
	global mirror_subprocess
	if mirror_subprocess is not None:
		os.killpg(os.getpgid(mirror_subprocess.pid), signal.SIGTERM)
		#mirror_subprocess.terminate()
def mirror_callback():
	kill_all()
	print "Running Mirror!"
	global mirror_subprocess
	
	path = rospack.get_path('basic_gui')# get the file path for rospy_tutorials
	command = path + '/src/mirror_left.py'
	#print command
	mirror_subprocess = subprocess.Popen(command, stdout=subprocess.PIPE,  shell=True,preexec_fn=os.setpgrp)


def Tucking_callback():
	kill_all()
	print "Tucking Baxters Arms. Goodbye!"
	path = rospack.get_path('baxter_tools')# get the file path for rospy_tutorials
	command = path + '/scripts/tuck_arms.py -t'
	subprocess.call(command, shell=True)
def Untucking_callback():
	kill_all()
	print "Untucking Baxter's arms. Soon he will be ready to go!!"
	path = rospack.get_path('baxter_tools')# get the file path for rospy_tutorials
	command = path + '/scripts/tuck_arms.py -u'
	subprocess.call(command, shell=True)

def TurnOffUltrasonicSensors():
	#kill_all()
	print "Turning Off Ultrasonic Sensor!"
	path = rospack.get_path('basic_gui')# get the file path for rospy_tutorials
	command = path + '/src/ShutOffUltrasonicSensors.py --data 0'
	subprocess.call(command, shell=True)
	print "Ultrasonic Sensors Off!\n"
def TurnOnUltrasonicSensors():
	#kill_all()
	print "Turning On Ultrasonic Sensor!"
	path = rospack.get_path('basic_gui')# get the file path for rospy_tutorials
	command = path + '/src/ShutOffUltrasonicSensors.py --data 4095'
	subprocess.call(command, shell=True)
	print "Ultrasonic Sensors On!\n"

# main/root window stuff
WINDOW_SIZE = "550x120"

root = tk.Tk()
root.title("Baxter Control")
root.geometry(WINDOW_SIZE)


# frame stuff (helps makes things neater in the main/root window
frame = tk.Frame(root)
frame.pack()

middleframe = tk.Frame(root)
middleframe.pack( )

thirdframe = tk.Frame(root)
thirdframe.pack( )

bottomframe = tk.Frame(root)
bottomframe.pack( side = tk.BOTTOM )



greenbutton = tk.Button(frame, text="Untuck Baxter's Arm", fg = "black", command=Untucking_callback)
greenbutton.pack(side=tk.LEFT)

greenbutton = tk.Button(frame, text="Tuck Baxter's Arm", fg = "black", command=Tucking_callback)
greenbutton.pack(side=tk.LEFT)

greenbutton = tk.Button(middleframe, text="Enable Ultrasonic Sensors", fg = "black", command=TurnOnUltrasonicSensors)
greenbutton.pack(side=tk.LEFT)

greenbutton = tk.Button(middleframe, text="Disable Ultrasonic Sensors", fg = "black", command=TurnOffUltrasonicSensors)
greenbutton.pack(side=tk.LEFT)

greenbutton = tk.Button(thirdframe, text="Mirror Left Arm", fg = "black", command=mirror_callback)
greenbutton.pack(side=tk.LEFT)

greenbutton = tk.Button(bottomframe, text="Stop Programs", fg = "red", command=kill_all)
greenbutton.pack(side=tk.LEFT)

# opens the main/root window and everything starts working!! (ideally)
root.mainloop()

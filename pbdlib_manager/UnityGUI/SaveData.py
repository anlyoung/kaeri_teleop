#!/usr/bin/env python

import Tkinter as tk
import rospkg,os
from distutils.dir_util import copy_tree
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

def SaveData():
    print("Test")
    currentDataFolder = currentData.get()
    if not os.path.exists(currentDataFolder):
        print("Could not find folder at: " + currentDataFolder)
        return
    new_path = SaveDataButton.get()
    if not os.path.exists(new_path): os.makedirs(new_path)
    else:
        print("Path already exist at: "+new_path)
        return
    print("Data copied from %s to %s" % (currentDataFolder, new_path))
    copy_tree(currentDataFolder, new_path)
    set_defualts()

WINDOW_SIZE = "800x90"

large_font = ('Verdana',10)

master = tk.Tk()
master.title("VR Baxter Demo")
master.geometry(WINDOW_SIZE)

tk.Label(master, text="Current Data",font=large_font).grid(row=0)
tk.Label(master, text="Where to Save",font=large_font).grid(row=1)

currentData = tk.Entry(master,width="70")
SaveDataButton = tk.Entry(master,width="70")

currentData.grid(row=0, column=1)
SaveDataButton.grid(row=1, column=1)

def set_defualts():
    path = rospack.get_path('pbdlib_manager')# get the file path for rospy_tutorials
    default_path = path + "/Test/Unity"
    new_path = path + "/Test/Unity_"
    n = 1
    while os.path.isdir(new_path + str(n)):
        n+=1
    currentData.delete(0,tk.END)
    currentData.insert(0,default_path)
    SaveDataButton.delete(0,tk.END)
    SaveDataButton.insert(0,new_path+ str(n))
set_defualts()

tk.Button(master, 
          text='Save Data', 
          command=SaveData).grid(row=3,  column=0, sticky=tk.W,   pady=4)

tk.mainloop()
#!/usr/bin/env python
#This is a class to launch and run nodes. See the example at the bottom
import roslaunch, rospkg, os,rospy
from time import sleep


class CustomNodeRun(object):
    def __init__(self,package,executable,args = ''):
        self.package = package
        self.executable = executable 
        self.args = args
        self.node = roslaunch.core.Node(package, executable, output = "screen",args=self.args)
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        self.process = None

    def start(self):
        self.process = self.launch.launch(self.node)
        return self

    def stop(self):
        if self.process is not None:
            if self.process.is_alive():
                self.process.stop()

    def is_alive(self):
        if self.process is None:return False
        return self.process.is_alive() 


class CustomLaunchRun(object):
    def __init__(self,package,executable):
        self.package = package
        self.executable = executable
        rospack = rospkg.RosPack()

        path = rospack.get_path(self.package)

        def find(name, path):
            for root, dirs, files in os.walk(path):
                if name in files:
                    return os.path.join(root, name)
            return None

        self.launchFile = find(self.executable,path)
        if self.launchFile == None:  raise Exception("Could not find %s in package %s"%(self.executable,self.package))

        self.uuid = None
        self.launch = None

    def start(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.launchFile])
        self.launch.start()
        return self
    def stop(self):
        if self.launch is not None: self.launch.shutdown()


"""
#Example Below
if __name__ == '__main__':
    #Creating a launch file
    print("Testing")
    testLaunch = CustomLaunchRun("kinfu", "kinect_single.launch")
    sleep(5)
    
    #How to start a launch
    print("Starting Launch")
    testLaunch.start()
    sleep(5)
    
    #how to Stop a Launch
    print("Stopping Launch")
    testLaunch.stop()


    #Run a file

    #Start an executable 
    #Create a Node and start it in one command
    ControllerExtractionNode = CustomNodeRun('baxter_tools','tuck_arms.py','-u').start()
    print(ControllerExtractionNode.is_alive())#check if the program is still alive

    sleep(25)

    #Stoping the programs
    ControllerExtractionNode.stop()
    print(ControllerExtractionNode.is_alive())
"""
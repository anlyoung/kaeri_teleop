#!/usr/bin/env python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas
import rospy
import rospkg

rospack = rospkg.RosPack()

class PlotMotionPrimitives(object):
    def __init__(self, nbStates=6, nbVarPos=3, nbDeriv=3, dt=0.0167, nbSamples=5, nbData=175):
        self.initialize(nbStates, nbVarPos, nbDeriv, dt, nbSamples, nbData)

    def initialize(self, nbStates, nbVarPos, nbDeriv, dt, nbSamples, nbData):
        self._nbStates = nbStates
        self._nbVarPos = nbVarPos
        self._nbDeriv = nbDeriv
        self._dt = dt
        self._nbSamples = nbSamples
        self._nbData = nbData
        self._nbVar = self._nbVarPos * self._nbDeriv
        self.data = []
        
        self._defaultDemoDataPath = rospack.get_path('pbdlib_manager')

    def LoadDemoData(self, folderName, id = 1, category = 'Demo'):
        if(category == 'Demo'):
            datafileName = self._defaultDemoDataPath + folderName + "/" + str(id) + '.txt'
        elif(category == 'Result'):
            datafileName = self._defaultDemoDataPath + folderName + "/" + 'optimalData1.txt'

        self._folderName = folderName
        #print(datafileName)
        df = pandas.read_csv(datafileName, sep=",", header=None)
        #print(df.size/3)
        for n in range(df.size/3):
            df[n][0] = df[n][0] + 1.148
            df[n][1] = df[n][1] - 0.0216
            df[n][2] = df[n][2] + 0.15
        self.data.append(df)


    def PlotDemoData(self, fig, ax, id):
        xs = self.data[id].iloc[0].values
        ys = self.data[id].iloc[1].values

        if(self._nbVarPos == 3):
            zs = self.data[id].iloc[2].values
            plt.plot(xs, ys, zs)
        else:
            plt.plot(xs, ys)    
    
    def UpdateData(self):
        if True:
            a = 1

    def Test(self, folderName, fig):
        num_fig = 2
        #plot 1
        ax1 = fig.add_subplot(1, num_fig, 1, projection='3d')


        for i in range(self._nbSamples):
            self.LoadDemoData(folderName, id = i,  category = 'Demo')
            self.PlotDemoData(fig, ax1, id = i)

        plt.title("Demos Data") 
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')

        """ax1.set_xlim3d(0.4, .7)
        ax1.set_ylim3d(-.7,-.4)
        ax1.set_zlim3d(.1,.7)"""

        #print(ax1.axis3d((.2,.7,-.6,-.4,0,1)))
        #plot 2
        ax2 = fig.add_subplot(1, num_fig, 2, projection='3d')
        self.LoadDemoData(folderName, category = 'Result')
        self.PlotDemoData(fig, ax2, id = i)

        plt.title("Result Data") 
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')

        """ax2.set_xlim3d(0.4, .7)
        ax2.set_ylim3d(-.7,-.4)
        ax2.set_zlim3d(.1,.7)"""

        #print(self.data[4])        
        #plt.draw()
        #plt.pause(0.01)
        #plt.show()

        self.UpdateData()

def main():
    print("Initializing node... ")
    rospy.init_node("plot_motion_primitives")
    
    a = PlotMotionPrimitives()
    fig1 = plt.figure(0, figsize=plt.figaspect(0.4))
    a.Test('/Test/Unity', fig1)
    fig1.show()

    """b = PlotMotionPrimitives()
    fig2 = plt.figure(1, figsize=plt.figaspect(0.4))
    b.Test('/Test/RawPhantom3', fig2)
    fig2.show()"""

    plt.show()

    return 0

if __name__ == "__main__":
	main()

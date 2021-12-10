#!/usr/bin/env python
import yaml
import io
import rospkg
import pandas
import rospy
import numpy as np
import os
import argparse
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



rospack = rospkg.RosPack()

class AdaptivePrimitives(object):
    def __init__(self, package, path, file_name):# hand='LeftHand', motion='moving', motion_num='3'):
        self.initialize(package,path, file_name)

    def initialize(self,package, path, file_name):
        source_location = rospack.get_path(package)
        #self.data_location = os.path.abspath((os.path.abspath(source_location) + "/../../../LDRD_Data"))
        
        self.data = []
        #self._defaultDemoDataPath = self.data_location + '/experiments/MotionPrimitives/'
        self.datafileName = os.path.join(source_location, path , file_name) 
        #print("motion (motion_num)", motion + motion_num)
        print("datafileName", self.datafileName)

        #self.motion_num = motion_num
        #self.traj_type = motion
        
        self.df_extract = pandas.DataFrame()
        self.df_shifted = pandas.DataFrame()


    def LoadData(self):
        self.data = []

        df = pandas.read_csv(self.datafileName, sep=",", header=None)
        df.rename(columns = lambda x: str(x), inplace=True)
        df.rename(str, columns={'0': 'idx', '1': 'Name', '2' : 'Pose'}, inplace=True)
        #print('df', df)
        self.df_extract = df.loc[:, ~df.columns.isin(['idx', 'Pose'])]
        print('df', self.df_extract)

        return self.df_extract

    def GetExtractNormData(self, startPt, endPt):

        self.xs = []
        self.ys = []
        self.zs = []

        _df = self.df_extract#.loc[self.df_extract.Name == self.hand]
        _df.pop('Name')
        num_data = len(_df.columns)

        task_start = np.array([startPt[0], startPt[1], startPt[2]])
        task_end = np.array([endPt[0], endPt[1], endPt[2]])
        disp_task =  task_end - task_start

        hand_start = np.array([_df.iloc[0, 0], _df.iloc[1, 0], _df.iloc[2, 0]])
        hand_end = np.array([_df.iloc[0, num_data-1], _df.iloc[1, num_data-1], _df.iloc[2, num_data-1]])
        disp_model = hand_end - hand_start

        for i in range(num_data):
            hand_data_i = np.array([_df.iloc[0, i], _df.iloc[1, i], _df.iloc[2, i]]) 
            ref_data = np.multiply(np.true_divide(hand_data_i - hand_start, disp_model), disp_task) + task_start

            x = float(ref_data[0])
            y = float(ref_data[1])
            z = float(ref_data[2])

            self.xs.append(x)
            self.ys.append(y)
            self.zs.append(z)
            print("[x,y,z] : ", x, y, z)

        data = {'x':self.xs,'y':self.ys,'z':self.zs}
        self.df_shifted = pandas.DataFrame(data)
        


        return self.xs, self.ys, self.zs, num_data
    def Plot(self):
        mpl.rcParams['legend.fontsize'] = 10

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
        z = np.linspace(-2, 2, 100)
        r = z**2 + 1
        x = r * np.sin(theta)
        y = r * np.cos(theta)
        ax.plot(self.xs, self.ys, self.zs)
        ax.legend()

        plt.title("Result Data") 
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.show()

def main():
    print("Initializing node... ")
    rospy.init_node("Adative_Primitives")

    parser = argparse.ArgumentParser(description='Adative_Primitives')

    """parser.add_argument('-t', '--target_hand',
                        type=str,
                        help="Target Hand ('LeftHand' or 'RightHand')",
                        dest='target_hand',
                        default='LeftHand'             # Only Allow Left_Hand And Right_Hand
                        )

    parser.add_argument('-i', '--motion_num',
                        type=str,
                        help='number of try indices',
                        dest='motion_num',
                        default='3'
                        )

    parser.add_argument('-m', '--motion',
                        type=str,
                        help='motion of interest',
                        dest='motion',
                        default='moving'
                        )"""

    args = parser.parse_args()

    a = AdaptivePrimitives("pbdlib_manager","Test/Unity","optimalData1.txt")
    print(a.LoadData())
    a.GetExtractNormData([0,0,0],[0.01,0.01,0.01])
    a.Plot()
    print("Done")
    print(a.df_shifted)
    print("Done")

    return 0

if __name__ == "__main__":
    main()

#!/usr/bin/env python

import numpy as np
import transformations as tf

import rospkg
import rospy
import tf as tf_for_pos
import geometry_msgs.msg
from ConfigParser import SafeConfigParser

import argparse
parser = argparse.ArgumentParser(description='Finds the position and orientaion of the kinect')
parser.add_argument('-f', '--folder', dest = "folder", action="store", type =str, default=None,help = "where the program looks for calib.ini. Default is in the package calibrate_kinnect in the folder data/last_test/.")
parser.add_argument('-s', '--save', dest = "save", action="store_false", default=True, help = "Disables the storing the position to the file kinect_pos_config.txt.")

args = parser.parse_args()


def get_tf(a,b):
    max_wait= 50
    trans,rot = None,None
    rospy.init_node('baxter_tf_listener')
    listener = tf_for_pos.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(a,b,  rospy.Time(0))
            return True,trans,rot
        except (tf_for_pos.LookupException, tf_for_pos.ConnectivityException, tf_for_pos.ExtrapolationException):
            max_wait -=1
        if max_wait == 0:
            print("Could not find the tf of "+a+ " and " + b + " the camera. Please try to manually calibrate it.")
            return False,None,None
        rate.sleep()


# w2l: world to left hand camera
# l2c: left hand camera to camera (kinect)
# c2k: camera coordinate to kinect link coordinate
# w2k: target

# Left Hand Camera pose and orintation from rviz/tf/left_hand_camera
# from TF/left_hand_camera in Rviz
grabbed,trans,rot = get_tf('/base','/left_hand_camera')
if not grabbed:
    exit()

xl_t = trans[0]
yl_t = trans[1]
zl_t = trans[2]

xl_q = rot[0]
yl_q = rot[1]
zl_q = rot[2]
wl_q = rot[3]


"""
#Could be manually entered from Rviz below
xl_t = 0.571
yl_t = 0.361
zl_t = 1.00

xl_q = 0.642
yl_q = 0.556
zl_q = 0.253
wl_q = 0.462#"""


print("""The left hand camera pose is:
Position:
    x = %f
    y = %f
    z = %f
Orientation:
    x = %f
    y = %f
    z = %f
    w = %f\n\n"""%(xl_t,yl_t,zl_t,xl_q,yl_q,zl_q,wl_q))

main_path = None
# Relative pose and orientation of right_camera (kinect) to
# left camera (left_hand_camera) from kinect_calibration report
if args.folder is None:
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # get the file path for rospy_tutorials
    path = rospack.get_path('calibrate_kinnect') + '/data/last_test/'
else:
    if args.folder[-1] == "/":
        path = args.folder
    else:
        path = args.folder + "/"

parser = SafeConfigParser()
try:
    parser.read(path + 'calib.ini')
    print("Reading from file: "+ path + 'calib.ini')
    main_path = path
except:
    print("Could not read "+path + 'calib.ini')

pose = parser.get('CAMERA_PARAMS_LEFT2RIGHT_POSE', 'pose_quaternion')
pose = pose [1:-1]#remove brackets
li = list(pose.split(" "))#split up string
pose = [float(a) for a in li]#convert to integer

xk_t = pose[0]
yk_t = pose[1]
zk_t = pose[2]

xk_q = pose[4]
yk_q = pose[5]
zk_q = pose[6]
wk_q = pose[3]#w comes first in the calib.ini file


"""
#Could be manually entered from calib.ini below
xk_t = 0.510186 
yk_t =  -0.163180
zk_t =   0.181729

xk_q =  -0.082525 
yk_q =  -0.325430
zk_q =  -0.188804
wk_q =  0.922842#"""

print("""The relative camera pose is:
Position:
    x = %f
    y = %f
    z = %f
Orientation:
    x = %f
    y = %f
    z = %f
    w = %f\n\n"""%(xk_t,yk_t,zk_t,xk_q,yk_q,zk_q,wk_q))


#  Base of Kinect relative to RGB (	estimate) from actual kinect camera
x_rgb = -.03
y_rgb = -.023
z_rgb = 0


# translation
w2l_t = np.array([xl_t,yl_t,zl_t])
print "The World to Left Hand Translation values are:\n "			
print w2l_t
print "\n"

# rotation (quaternion)
w2l_q = np.array([wl_q,xl_q,yl_q,zl_q])		
w2l_m = tf.quaternion_matrix(w2l_q)
print "World to Left Hand Quaternion matrix is:\n"
print w2l_m
print "\n"

for i in range(3): w2l_m[i,3] = w2l_t[i]
print "New World to Left Hand matrix is: \n"
print w2l_m
print '\n'

l2c_t = np.array([xk_t,yk_t,zk_t])			# translation
l2c_q = np.array([wk_q,xk_q,yk_q,zk_q])		# rotation (quaternion)
l2c_m = tf.quaternion_matrix(l2c_q)
print "The Left Hand to Kinect Quaternion matrix:\n"
print l2c_m
print '\n'

for i in range(3): l2c_m[i,3] = l2c_t[i]
print "New Left Hand to Kinect matrix is:\n"
print l2c_m
print '\n'

# x, y, z: translation between camera coordinate and kinect link coordinate
c2k_m = np.array([[0,-1,0,x_rgb],[0,0,-1,y_rgb],[1,0,0,z_rgb],[0,0,0,1]])
print "RGB Cam to Kinect Base:\n"
print c2k_m
print '\n'

w2k_m = np.dot(w2l_m, np.dot(l2c_m, c2k_m))
w2k_q = w2k_m

print "World to Kinect matrix:\n"
print w2k_m
print '\n'
w2k_t = np.array([w2k_m[0,3],w2k_m[1,3],w2k_m[2,3]])

print "World to Kinect quaternion:\n"
print w2k_q
print '\n'

for i in range(3): w2k_q[i,3] = 0
print "New World to Kinect quaternion matrix:\n"
print w2k_q
print '\n'

w2k_q = tf.quaternion_from_matrix(w2k_q)	# wxyz sequence
#w2k_t = np.array([w2k_m[0,3],w2k_m[1,3],w2k_m[2,3]])

#print w2k_m
print '\n'
print "Translation values are:\n"
print "  x\t|\ty\t|\tz"
print w2k_t
print '\n'

print "Quaternion values are:\n"
print "  w\t|\tx\t|\ty\t|\tz"
print w2k_q
print '\n'

w2k_t_str = str(w2k_t[0])+", "+str(w2k_t[1])+", "+str(w2k_t[2])+",\n"
w2k_q_str = str(w2k_q[1])+", "+str(w2k_q[2])+", "+str(w2k_q[3])+", "+str(w2k_q[0])+",\n"

if args.save:
    save = True
    file_name = None
    try:
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        file_name = rospack.get_path("kinfu") + "/src/RGBD_link_tf_broadcaster/kinect_pos_config.txt"
    except:
        print("Could not find package kinfu package. Translation values not saved.")
        save = False

    if save:
        try:
            f= open(file_name,"w+")
            f.write(w2k_t_str)
            f.write(w2k_q_str)
            f.write("# position x, position y, position z\n# orientation x, orientation y,orientation z,orientation w,")
            f.close()
            print("Value Saved")
            print("Using the calib.ini from:\n"+path+"\n")
            print("Translation values are to be saved to "+ file_name +". This updates ROS.\n")
        except:
            print("Error saving to "+ file_name)

if main_path is not None:
    try:
        file_name = main_path + "kinect_pos_config.txt"
        f= open(file_name,"w+")
        f.write(w2k_t_str)
        f.write(w2k_q_str)
        f.write("# position x, position y, position z\n# orientation x, orientation y,orientation z,orientation w,")
        f.close()
        print("Translation values are also stored FOR Reference to "+ file_name +". This does not update on ROS.\n")
    except:
        print("Error saving to "+ file_name)
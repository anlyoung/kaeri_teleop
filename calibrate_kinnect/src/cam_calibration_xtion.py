#!/usr/bin/env python

import numpy as np
import transformations as tf

# w2l: world to left hand camera
# l2c: left hand camera to camera (kinect)
# c2k: camera coordinate to kinect link coordinate
# w2k: target

# Right Hand Camera Pose and Orientation from tf
xl_t = 0.378
yl_t = -0.487
zl_t = 0.925

xl_q = 0.704
yl_q = -0.483
zl_q = 0.2403
wl_q = -0.460

# Xtion Position and Orientation from Report 
xk_t = -0.848723
yk_t = -0.115439
zk_t = 0.253552

xk_q = -0.018925
yk_q = 0.469416
zk_q = 0.141404
wk_q = 0.871375

#  Base of Kinect relative to RGB (	estimate) from actual kinect camera
#x_rgb = 0.0125
#y_rgb = 0.010
#z_rgb = -0.050
#x_rgb = -0.0125
x_rgb = 0
#y_rgb = -0.023
y_rgb = 0
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



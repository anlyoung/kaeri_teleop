# Setting up the Omni

# Lines 5-8 Enable the usb port to read data from the omni
sudo ln /dev/fw0 /dev/raw1394		
sudo chmod 0777 /dev/raw1394  
sudo chmod 0777 /dev/fw0
sudo chmod 0777 /dev/fw1

# Lines 11-12 are a demo for the omni to make sure it is working
cd /usr/share/3DTouch/examples/HD/console/FrictionlessSphere
./FrictionlessSphere

#-----------------------------------------------------------
# Running Baxter 1 PC Simulator

cd ~/catkin_ws
./sim_baxter_desktop.sh

roslaunch teleop single_pc_simulation.launch

#-----------------------------------------------------------
# Baxter 1 PC Physical Robot

cd ~/catkin_ws
./baxter.sh

roslaunch teleop demo_teleop_only_wall_tower.launch

#-----------------------------------------------------------
# Baxter with 2 PC Physical Robot

cd ~/catkin_ws
./baxter.sh

# Launch for PC with the OMNI
roslaunch teleop demo_teleop_wall_pc.launch

# Launch for PC with Kinfu and Kinect
roslaunch teleop demo_teleop_dell_tower.launch

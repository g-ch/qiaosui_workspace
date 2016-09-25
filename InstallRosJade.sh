#!/bin/sh
#author: Clarence Chan, Sept 23th, 2016

#keys, environment
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
echo "Keys settled. Install ROS now!"

#ros-jade
sudo apt-get install ros-jade-desktop-full
sed -i '$a source /opt/ros/jade/setup.bash' ~/.bashrc
echo "Ros Jade installed successfully. Install some accessories."

#archives
source /opt/ros/jade/setup.bash
sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall
echo "Accessories installed successfully. Start building workspace."

#workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
sed -i '$a source ~/catkin_ws/devel/setup.bash ' ~/.bashrc
echo "Finished! You can use catkin_make now."


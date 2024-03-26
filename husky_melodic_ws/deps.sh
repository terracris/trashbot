#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep

sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y

sudo apt-get install ros-melodic-velodyne-gazebo-plugins -y
sudo apt-get install ros-melodic-twist-mux -y
sudo apt-get install ros-melodic-velodyne-pointcloud -y
sudo apt-get install ros-melodic-dwa-local-planner -y

sudo apt-get install ros-melodic-pointcloud-to-laserscan -y
sudo apt-get install ros-melodic-teleop-twist-joy -y
sudo apt-get install ros-melodic-urg-node -y
sudo apt-get install ros-melodic-navfn -y

sudo apt-get install ros-melodic-robot-localization ros-melodic-navigation ros-melodic-velodyne-description -y

sudo apt-get install ros-melodic-slam-gmapping -y
sudo apt-get install ros-melodic-sick-scan -y
sudo apt-get install ros-melodic-multimaster-launch -y
sudo apt-get install ros-melodic-sick-scan -y
sudo apt-get install ros-melodic-um7 -y
sudo apt-get install ros-melodic-rviz-imu-plugin -y

sudo apt-get install ros-melodic-hector-gazebo-plugins -y
sudo apt-get install ros-melodic-joint-trajectory-controller -y
sudo apt-get install ros-melodic-um6 -y
sudo apt-get install ros-melodic-lms1xx -y
sudo apt-get install ros-melodic-joint-state-publisher-gui -y

sudo apt-get install ros-melodic-interactive-marker-twist-server -y
sudo apt-get install ros-melodic-robot-upstart -y
sudo apt-get install ros-melodic-realsense2-description -y
sudo apt-get install ros-melodic-realsense2-camera -y

sudo apt-get install ros-melodic-nmea-navsat-driver -y
sudo apt-get install ros-melodic-nmea-comms -y
sudo apt-get install ros-melodic-microstrain-mips -y
sudo apt-get install ros-melodic-microstrain-3dmgx2-imu -y
sudo apt-get install ros-melodic-imu-transformer -y
sudo apt-get install ros-melodic-imu-filter-madgwick -y


cd ~/trashbot/husky_melodic_ws
catkin_make

#!/bin/bash

## ros public key change
# sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

sudo apt -y install python-pip
sudo apt -y install ros-kinetic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash

sudo apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

git submodule init
git submodule update

sudo apt -y install ros-kinetic-openslam-gmapping
sudo apt -y install ros-kinetic-geodesy
sudo apt -y install ros-kinetic-geometry-msgs 
sudo apt -y install ros-kinetic-geographic-msgs
sudo apt -y install ros-kinetic-bfl


sudo apt -y install python-visual

cd ~/ISCC_2019/ && catkin_make

echo "source ~/ISCC_2019/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

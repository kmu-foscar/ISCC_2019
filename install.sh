#!/bin/sh

## ros public key change
# sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
# sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt install python-pip
sudo apt install ros-kinetic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

git submodule init
git submodule update

sudo apt install ros-kinetic-openslam-gmapping
sudo apt install ros-kinetic-geodesy
sudo apt install ros-kinetic-geometry-msgs 
sudo apt install ros-kinetic-geographic-msgs
sudo apt install ros-kinetic-bfl


sudo apt install python-visual

echo "source ~/ISCC_2019/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

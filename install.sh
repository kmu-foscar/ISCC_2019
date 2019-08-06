#!/bin/sh

git submodule init
git submodule update


sudo apt install ros-kinetic-geometry-msgs 
sudo apt install ros-kinetic-geographic-msgs
sudo apt install ros-kinetic-bfl


sudo apt install python-visual

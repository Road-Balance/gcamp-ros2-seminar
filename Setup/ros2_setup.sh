#!/bin/bash -e

# Reference sites
# https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

sudo apt-get update
sudo apt-get upgrade

sudo apt-get install curl

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo -e "${GREEN}==== Add the ROS repository ====${NC}"
if [ ! -e /etc/apt/sources.list.d/ros2-latest.list ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
fi

echo -e "${GREEN}==== Update the package ====${NC}"
sudo apt-get update

echo -e "${GREEN}==== Installing ROS and ROS Packages ====${NC}"
sudo apt-get install -y ros-eloquent-desktop

echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install -y python3-pip
pip3 install -U argcomplete

echo -e "${GREEN}==== Installing Colcon Build system ====${NC}"
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo -e "${GREEN}==== Update the package Again ====${NC}"
sudo apt update
sudo apt-get install -y python3-colcon-common-extensions

sudo apt-get install ros-eloquent-turtlesim -y
sudo apt-get install ros-eloquent-slam-toolbox -y
sudo apt-get install ros-eloquent-camera-info-manager -y
sudo apt-get install ros-eloquent-teleop-twist-keyboard -y
sudo apt-get install python-rosdep -y

echo -e "${GREEN}==== Create ROS 2 Workspace ====${NC}"
mkdir -p ~/ros2_seminar_ws/src
cd ~/ros2_seminar_ws/src

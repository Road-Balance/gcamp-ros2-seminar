# Copyright 2021 @ Nanosaur
# 
# This Setup file contains few utils for initial jetson setup
# 
# * Jetson Cooling fan control from jugfk [https://github.com/jugfk/jetson-fan-ctl]
# * Extend Memory Swap Area from jetsonhacksnano [https://github.com/jetsonhacksnano/installSwapfile]
#
# * Fill free to use this template 

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}==== Apt install ====${NC}"
sudo apt-get update && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y libglew-dev glew-utils libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev libglib2.0-dev \
    nvidia-tensorrt && \
    rm -rf /var/lib/apt/lists/*

echo -e "${GREEN}==== Jeton Utils install ====${NC}"
cd ~/Documents
cd ~/Downloads
if [ -d ./jetson-inference ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf jetson-inference
fi

sudo apt-get install git cmake
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference

echo -e "${GREEN}==== Jeton Interface Build (This takes for a while) ====${NC}"
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig

pip3 install wheel
pip3 install -U wstool

echo -e "${GREEN}==== ROS 2 Dependencies Install ====${NC}"
sudo apt-get install ros-eloquent-vision-msgs \
                       ros-eloquent-launch-xml \
                       ros-eloquent-launch-yaml \
                       python3-colcon-common-extensions


echo -e "${GREEN}==== Clone & Build Package ====${NC}"

cd ~/ros2_seminar_ws/src
if [ -d ./ros_deep_learning ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf ros_deep_learning
fi

git clone https://github.com/dusty-nv/ros_deep_learning
cd ~/ros2_seminar_ws
colcon build
source install/local_setup.bash 
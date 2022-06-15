#!/bin/bash -e

# Reference sites
# trt_pose repo : https://github.com/NVIDIA-AI-IOT/trt_pose

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}==== Installing Swapfile ====${NC}"

cd ~/Downloads
if [ -d ./installSwapfile ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf installSwapfile
fi

git clone https://github.com/jetsonhacksnano/installSwapfile
cd installSwapfile
./installSwapfile.sh


echo -e "${GREEN}==== Install PyTorch and Torchvision ====${NC}"
echo -e "${GREEN}==== PyTorch 1.8.0 Install  ====${NC}"

cd ~/Downloads
if [ -d ./torch-1.8.0-cp36-cp36m-linux_aarch64.whl ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf torch-1.8.0-cp36-cp36m-linux_aarch64.whl
fi
wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl \
    -O torch-1.8.0-cp36-cp36m-linux_aarch64.whl
sudo apt-get install python3-pip libopenblas-base libopenmpi-dev -y
pip3 install Cython
pip3 install numpy torch-1.8.0-cp36-cp36m-linux_aarch64.whl
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev -y

echo -e "${GREEN}==== Torchvision 0.9.0 Install  ====${NC}"
cd ~/Downloads
if [ -d ./torchvision ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf torchvision
fi

git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.9.0
python3 setup.py install --user

echo -e "${GREEN}==== torch2trt Install  ====${NC}"
cd ~/Downloads
if [ -d ./torch2trt ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf torch2trt
fi

git clone -b jp4.6_tensorrt8 https://github.com/chitoku/torch2trt.git
cd torch2trt
sudo python3 setup.py install --plugins


echo -e "${GREEN}==== Install other miscellaneous packages  ====${NC}"
sudo pip3 install tqdm cython pycocotools
sudo apt-get install python3-matplotlib

echo -e "${GREEN}==== Install trt_pose  ====${NC}"
cd ~/Downloads
if [ -d ./trt_pose ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf trt_pose
fi

git clone https://github.com/NVIDIA-AI-IOT/trt_pose
cd trt_pose
sudo python3 setup.py install

echo -e "${GREEN}==== Download provided Models ====${NC}"
cd tasks/human_pose/
wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1XYDdCUdiF2xxx4rznmLb62SdOUZuoNbd' \
    -O resnet18_baseline_att_224x224_A_epoch_249.pth
wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=13FkJkx7evQ1WwP54UmdiDXWyFMY1OxDU' \
    -O densenet121_baseline_att_256x256_B
    
echo -e "${GREEN}==== Clone and Build trt_pose_ros2 pkg ====${NC}"
cd ~/ros2_seminar_ws/src
git clone https://github.com/kimsooyoung/ros2_trt_pose.git
cd ~/ros2_seminar_ws
cba


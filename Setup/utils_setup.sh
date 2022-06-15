# Copyright 2021 @ RoadBalance
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

echo -e "${GREEN}==== Apt Update and Upgrade ====${NC}"
sudo apt update && sudo apt upgrade -y

echo -e "${GREEN}==== Installing jetson-fan-ctrl ====${NC}"
cd ~/Downloads
if [ -d ./jetson-fan-ctl ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf jetson-fan-ctl
fi

git clone https://github.com/jetsonworld/jetson-fan-ctl.git
cd jetson-fan-ctl
sudo sh install.sh

if [ ! $? -eq 0 ]
then
	echo -e "${RED} jetson-fan-ctl Install failed ${NC}"
	exit 1
fi

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

echo -e "${GREEN}==== Installing LXDE Setup ====${NC}"

cd ~/Downloads
if [ -d ./installLXDE ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf installLXDE
fi

git clone https://github.com/jetsonhacks/installLXDE.git
cd installLXDE
./installLXDE.sh

echo -e "${GREEN}==== Installing Terminator ====${NC}"

sudo apt install terminator -y

echo -e "${GREEN}==== Useful Aliases Setup ====${NC}"

echo -e "alias eb='gedit ~/.bashrc'" | sudo tee -a ~/.bashrc
echo -e "alias sb='source ~/.bashrc'" | sudo tee -a ~/.bashrc
echo -e "alias cba='colcon build --symlink-install'" | sudo tee -a ~/.bashrc
echo -e "alias cbp='colcon build --symlink-install --packages-select'" | sudo tee -a ~/.bashrc
echo -e "alias roseloq='source /opt/ros/eloquent/setup.bash && source ~/ros2_seminar_ws/install/local_setup.bash'" | sudo tee -a ~/.bashrc
echo -e "source /usr/share/colcon_cd/function/colcon_cd.sh" | sudo tee -a ~/.bashrc
echo -e "export _colcon_cd_root=~/ros2_seminar_ws" | sudo tee -a ~/.bashrc


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
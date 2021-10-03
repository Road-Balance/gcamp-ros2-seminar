GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

cd ~/Downloads
if [ -d ./installVSCode ]
then
	echo -e "${RED} delete exist folder ${NC}"
	rm -rf installVSCode
fi

git clone https://github.com/JetsonHacksNano/installVSCode.git
cd installVSCode
./installVSCode.sh

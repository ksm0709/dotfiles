#!/bin/bash

sudo apt-get install git curl wget -y

# echo 색상 목록
RED='\033[1;31m'
GREEN='\033[1;32m'
ORANGE='\033[1;33m'
BLUE='\033[1;34m'
NC='\033[0m'

# 정보 표시
echo
echo -e "${BLUE}>> ${ORANGE} Setup Custom Configs ... "
echo


# args 없으면 종료
if [ $# -lt 1 ]; then
  echo -e "${BLUE} ** USAGE"

  echo -e "    ./setup.sh vim tmux   : Install vim and tmux"
  echo -e "    ./setup.sh vim        : Install vim only"
  echo
  echo -e "${BLUE} Available options : ${GREEN} vim, tmux, ros"
  exit 1
fi 

# args 배열로 받아옴
args=("$@")

# 각 모듈 폴더로 접근해서 setup.sh 수행
for ((i=0;i<$#;i++))
do
  module=${args[$i]}

  echo -e "${BLUE}>> ${ORANGE} Setting [ ${module} ] ... ${NC}"
  
  cd ${module}

  ./setup.sh

  cd ..

  echo -e "${BLUE}>> ${GREEN} Setting [ ${module} ] Done !"

done


# 종료 메세지
echo
echo -e "${BLUE}>> ${GREEN} Setup Done !"
echo


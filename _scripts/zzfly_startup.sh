#!/bin/bash

SH_DIR=$(dirname $0)
WS_DIR="${SH_DIR}/../.."

RUN_DATE=`date +%Y%m%d_%H%M%S`

function getArrItemIdx(){
local arr=$1
local item=$2
local index=1
for i in ${arr[*]}
do
  if [[ $item == $i ]]
    then
    echo $index
    return
  fi
  index=$(( $index + 1 ))
done
}





# 集群参数
# mav_id=1
mav_id=`expr ${HOSTNAME:4:2} + 0`
mav_num=10
# swarm_array=(8 7 9)
# mav_num=${#swarm_array[*]}
# swarm_idx=`getArrItemIdx "${swarm_array[*]}" ${mav_id}`

echo "mav_id:${mav_id} mav_num:${mav_num}"
sleep 10s

# gnome-terminal -x bash -c "roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS0:921600";exec bash"
gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200";exec bash"
sleep 5s

# gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_t265_bridge.launch;exec bash"
gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_vicon.launch mav_id:=${mav_id};exec bash"
sleep 5s

gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_d435i_30hz.launch;exec bash"
sleep 5s


gnome-terminal --window -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_main.launch mav_id:=${mav_id} mav_num:=${mav_num};exec bash"
sleep 5s

sleep 10s

# gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;rosbag record ${topic} -o ${WS_DIR}/bag/zzfly_real_;exec bash"
gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;rosrun offboard_pkg bag_save.py mav_num:=${mav_num};exec bash"

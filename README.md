# planner_offboard_us

#### Description
fast planner using autonomic offboard algrithm with t265 . d435i . rplidar s1

This branch is master ! 

#### Installation
1. git clone https://gitee.com/greymaner/unionsys_core_algrithm_deb.git
2. cd unionsys_core_algrithm_deb/arm64
3. sudo dpkg -i unionsys_core-1.1.0-Linux.deb



#### Instructions

#! /bin/bash  
gnome-terminal -x bash -c "source $HOME/mavros_ws/devel/setup.bash;roslaunch px4_realsense_bridge bridge.launch;exec bash"  
sleep 10s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;roslaunch mavros px4.launch;exec bash"  
sleep 5s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;roslaunch plan_manage topo_replan.launch ;exec bash"  
sleep 5s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;roslaunch off_board_test main.launch ;exec bash"  
sleep 5s  
gnome-terminal -x bash -c  "source $HOME/mavros_ws/devel/setup.bash;cd $HOME/mavros_ws/src/fp_for_xgd/px4_offboard_controller/script;python duoji.py;exec bash"  
wait  
exit 0  



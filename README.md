# planner_offboard_us

#### Description
fast planner using autonomic offboard algrithm with t265 . d435i . rplidar s1

This branch is master ! 

#### Installation
1. git clone https://gitee.com/greymaner/unionsys_core_algrithm_deb.git
2. cd <path of your system type> sudo dpkg -i unionsys_core-1.1.0-Linux.deb
3.  catkin_make


#### Instructions

#! /bin/bash  
gnome-terminal -x bash -c   
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch px4_realsense_bridge bridge.launch;exec bash"  
sleep 10s  
gnome-terminal -x bash -c   
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch mavros px4.launch;exec bash"  
sleep 5s  
gnome-terminal -x bash -c    
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch plan_manage topo_replan.launch ;exec bash"  
sleep 5s  
gnome-terminal -x bash -c   
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch off_board_test main.launch ;exec bash"  
wait  
exit 0  




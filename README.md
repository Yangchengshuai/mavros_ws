# planner_offboard_us

#### Description
fast planner using autonomic offboard algrithm with t265 . d435i . rplidar s1

This branch is master ! 

#### Installation

1.  catkin_make


#### Instructions

#! /bin/bash  
gnome-terminal -x bash -c   
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch px4_realsense_bridge bridge.launch;exec bash"  
sleep 10s  
gnome-terminal -x bash -c   
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch mavros px4.launch;exec bash"  
sleep 10s  
gnome-terminal -x bash -c   
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch rplidar_ros rplidar_s1.launch ;exec bash"  
sleep 5s  
gnome-terminal -x bash -c    
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch plan_manage topo_replan.launch ;exec bash"  
sleep 5s  
gnome-terminal -x bash -c   
"source /home/zhou/mavros_ws/devel/setup.bash;roslaunch off_board_test main.launch ;exec bash"  
wait  
exit 0  




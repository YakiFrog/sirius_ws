#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
while : ;do
    read -p "Press [Enter] key to start waypoint_nav..."
    echo "Input waypoint file name (default: map.yaml): "
    read waypoint_file
    echo "Input waypoint number (default: 1): "
    read waypoint_number
    count="${waypoint_number:-1}"
    source install/setup.bash
    ros2 run sirius_navigation move_goal.py --file "$waypoint_file" --count "$count" 
done
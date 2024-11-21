#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start waypoint_nav..."
    echo "Input waypoint number (default: 1): "
    read waypoint_number
    count="${waypoint_number:-1}"
    source install/setup.bash
    ros2 run sirius_navigation move_goal.py --count "$count"
done
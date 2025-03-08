#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
while : ;do
    read -p "Press [Enter] key to start map save..."
    source install/setup.bash
    echo "Input map name (without .yaml): "
    read map_name
    ros2 run nav2_map_server map_saver_cli -f ~/sirius_ws/map_waypoints/map/$map_name
done
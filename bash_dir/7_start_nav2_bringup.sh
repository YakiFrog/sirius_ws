#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start nav2 bringup..."
    source install/setup.bash
    echo "Input map name (without .yaml): "
    read map_name
    ros2 launch nav2_bringup bringup_launch.py map:=/home/sirius24/sirius_ws/map_waypoints/map/$map_name.yaml params_file:=/home/sirius24/sirius_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml
done

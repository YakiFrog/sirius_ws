#!/bin/sh
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
while : ;do
    read -p "Press [Enter] key to start nav2 bringup..."
    cd ~/sirius_ws
    source install/setup.bash
    echo "Input map name (without .yaml): "
    read map_name
    ros2 launch nav2_bringup bringup_launch.py map:=~/sirius_ws/map_waypoints/map/$map_name.yaml
done
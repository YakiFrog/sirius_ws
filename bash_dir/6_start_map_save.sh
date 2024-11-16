#!/bin/sh
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
while : ;do
    read -p "Press [Enter] key to start map save..."
    cd ~/sirius_ws
    source install/setup.bash
    echo "Input map name (without .yaml): "
    read map_name
    ros2 run nav2_map_server map_saver_cli -f ~/sirius_ws/map_waypoints/map/$map_name
done
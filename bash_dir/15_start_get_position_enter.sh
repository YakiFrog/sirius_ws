#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start get_position_enter..."
    source install/setup.bash
    ros2 run sirius_navigation get_position_enter.py
done
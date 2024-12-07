#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start urg_node2..."
    source install/setup.bash
    sudo chmod 666 /dev/ttyACM0
    ros2 launch urg_node2 urg_node2.launch.py
done
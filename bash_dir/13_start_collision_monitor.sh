#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start collision_monitor..."
    source install/setup.bash
    ros2 launch nav2_collision_monitor collision_monitor_node.launch.py 
done
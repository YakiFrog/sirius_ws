#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ; do
    read -p "Press [Enter] key to start waypoint_nav..."
    echo "Input move_goal file name (default: move_goal.yaml): "
    read file_name
    yaml_file="${file_name:-move_goal.yaml}"
    source install/setup.bash
    python3 /home/sirius24/sirius_ws/src/sirius_navigation/scripts/move_goal.py --yaml_file "$yaml_file"
done


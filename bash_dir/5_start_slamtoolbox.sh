#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
while : ;do
    read -p "Press [Enter] key to start slamtoolbox..."
    source install/setup.bash
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:=/home/sirius24/sirius_ws/params/mapper_params_online_async.yaml
done
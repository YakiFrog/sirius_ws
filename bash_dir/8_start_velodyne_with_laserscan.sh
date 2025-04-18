#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_ws
export ROS_DOMAIN_ID=57
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

while : ;do
    read -p "Press [Enter] key to start velodyne..."
    source install/setup.bash
    
    # まずvelodyneの基本ノードを起動
    ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py #&
    
    # # 数秒待機してvelodyne_pointsトピックが利用可能になるのを待つ
    # sleep 5
    
    # # 2リング機能を持つvelodyne_laserscanノードを起動（パラメータを直接指定）
    # ros2 run velodyne_laserscan velodyne_laserscan_node --ros-args -p ring:=8 -p ring2:=5 -p resolution:=0.007
    
    # wait
done

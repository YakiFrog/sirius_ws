while : ;do
    read -p "Press [Enter] key to start tf2 static transform publisher 2..."
    cd ~/sirius_ws
    source install/setup.bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint velodyne
done
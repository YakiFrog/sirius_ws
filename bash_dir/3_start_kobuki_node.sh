while : ;do
    read -p "Press [Enter] key to start kobuki node..."
    cd ~/sirius_ws
    source install/setup.bash
    ros2 launch kobuki_node kobuki_node-launch.py
done
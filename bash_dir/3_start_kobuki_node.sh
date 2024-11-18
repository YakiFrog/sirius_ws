
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
while : ;do
    read -p "Press [Enter] key to start kobuki node..."
    source /home/sirius24/.bashrc
    cd ~/sirius_ws
    source install/setup.bash
    ros2 launch kobuki_node kobuki_node-launch.py
done
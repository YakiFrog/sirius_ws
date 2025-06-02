#!/bin/bash
# filepath: /home/sirius24/sirius_ws/bash_dir/21_start_face_animation.sh

# エラー時の処理設定
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2

# ワークスペースに移動
cd ~/sirius_ws

# ROS2環境設定
export ROS_DOMAIN_ID=57

# セットアップ実行
source install/setup.bash

echo "Face Animation Controller システムと顔アプリを起動します..."
echo "ブラウザで http://localhost:8080 にアクセスできます"
echo ""

# 顔アプリをバックグラウンドで起動
echo "顔アプリを起動中..."
/home/sirius24/Documents/sirius_face/sirius-face &
FACE_PID=$!

# 少し待機してからROS2システム起動
sleep 2

echo "Face Animation Controllerを起動中..."
# Face Animation システム起動
ros2 launch face_animation_controller face_animation_system.launch.py

# 終了処理：ROS2が終了したら顔アプリも終了
echo ""
echo "システムが終了しました。顔アプリも終了します..."
kill $FACE_PID 2>/dev/null

read -p "Press [Enter] key to close..."
#!/bin/bash

cd ~/sirius_ws
colcon build --packages-select velodyne_laserscan
source install/setup.bash
echo "velodyne_laserscan パッケージを再ビルドしました"

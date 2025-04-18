#!/bin/bash

# 設定ファイルを保存するディレクトリを作成
mkdir -p ~/sirius_ws/src/velodyne/velodyne_laserscan/config
mkdir -p ~/sirius_ws/install/velodyne_laserscan/share/velodyne_laserscan/config

# 既存の設定ファイルをコピー（または新規作成）
echo 'velodyne_laserscan_node:
    ros__parameters:
        ring: -1
        ring2: -1
        resolution: 0.007' > ~/sirius_ws/src/velodyne/velodyne_laserscan/config/default-velodyne_laserscan_node-params.yaml

# インストールディレクトリにもコピー
cp ~/sirius_ws/src/velodyne/velodyne_laserscan/config/default-velodyne_laserscan_node-params.yaml ~/sirius_ws/install/velodyne_laserscan/share/velodyne_laserscan/config/

echo "設定ファイルとディレクトリを作成しました"

#!/bin/bash

# configディレクトリを作成
mkdir -p ~/sirius_ws/src/velodyne/velodyne_laserscan/config

# パラメータファイルを作成
cat > ~/sirius_ws/src/velodyne/velodyne_laserscan/config/default-velodyne_laserscan_node-params.yaml << 'EOL'
velodyne_laserscan_node:
    ros__parameters:
        ring: -1
        ring2: -1
        resolution: 0.007
EOL

echo "Config directory and parameter file created successfully."

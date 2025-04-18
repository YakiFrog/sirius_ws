#!/bin/bash

# スクリプトに実行権限を与える
chmod +x ~/sirius_ws/create_config_dir.sh

# configディレクトリとファイルを作成する
~/sirius_ws/create_config_dir.sh

# パッケージをビルドする（上書きを許可）
cd ~/sirius_ws
colcon build --packages-select velodyne_laserscan --allow-overriding velodyne_laserscan

echo "Build complete. Don't forget to source the setup script with:"
echo "source ~/sirius_ws/install/setup.bash"

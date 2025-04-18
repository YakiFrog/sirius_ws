# いじれるパラメタ郡

## 1. nav2_params.yaml
```bash
/home/sirius24/sirius_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml
```

### 説明
---
Nav2のパラメタファイル．
ここに書かれているパラメタを変更することで，Nav2の挙動を変更することができる．
逆に書かれていないパラメタを追加すると，動かなくなる．

## 2. mapper_params_online_async.yaml
```bash
/home/sirius24/sirius_ws/src/slam_toolbox/config/mapper_params_online_async.yaml
```

### 説明
---
SLAM Toolboxのパラメタファイル．
リアルタイムで地図を作成するときのパラメタを設定する．


export CYCLONEDDS_URI=file:///home/sirius24/sirius_ws/cyclonedds.xml

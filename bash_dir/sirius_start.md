# sirius startup command

## 0. 座標系を合わせる
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link
```

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint velodyne
```

## 1. teleopをするとき
```bash
ros2 launch kobuki_node kobuki_node-launch.py
```

```bash
ros2 run kobuki_keyop kobuki_keyop_node --ros-args -p linear_vel_max:=1.0 -p angular_vel_step:=0.05 -p angular_vel_max:=0.5
```

## 2. slamをするとき
```bash
ros2 launch slam_toolbox online_async_launch.py
```

## 3. mapを保存するとき
```bash
ros2 run nav2_map_server map_saver_cli -f <path to save map>
```
例：/home/sirius/sirius_ws/map/$map_name

## 4. nav2を起動するとき
```bash
ros2 launch nav2_bringup bringup_launch.py map:=<path to map.yaml>
```

## 5. velodyneを起動するとき
```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py
```

### ubuntu上のvelodyneの設定
---
1. Wiredにvelodyneという名前で追加
2. IPv4
    - IPv4 Method: Manual
    - Address: 192.168.1.60
    - Netmask: 255.255.255.0
    - Gateway: 192.168.1.1

ほかはデフォルト

## 見るとき
```bash
screen /dev/ttyUSB0 115200
```
Ctrl + A -> K で閉じる

## 信地回転が狂う瞬間

```cpp
// 送信データの計算をしている場所
void DiffDrive::velocityCommands(const double &vx, const double &wz) {
  // vx [m/s]
  // wz [rad/s]
  velocity_mutex.lock();
  const double epsilon = 0.0001;

  // 特別なケース1: 直進
  if( std::abs(wz) < epsilon ) {
    radius = 0.0f;
    speed  = 1000.0f * vx;
    velocity_mutex.unlock();
    std::cout << "speed: " << speed << ", radius: " << radius << std::endl;
    return;
  }

  // 回転半径[mm]の計算
  radius = vx * 1000.0f / wz;

  // 特別なケース2: 超信地回転か，半径が1.0mm以下(ほぼ原点回転)
  if( std::abs(vx) < epsilon || std::abs(radius) <= 1.0f ) {
    speed  = 1000.0f * bias * wz / 2.0f;
    radius = 1.0f;
    velocity_mutex.unlock();
    std::cout << "speed: " << speed << ", radius: " << radius << std::endl;
    return;
  }

  // 通常の場合（移動 + 曲がる場合）
  // ここの計算のせいで信地回転が狂う
  if( radius > 0.0f ) { // 右回り
    speed  = (radius + 1000.0f * bias / 2.0f) * wz;
  } else { // 左回り
    speed  = (radius - 1000.0f * bias / 2.0f) * wz;
  }

  std::cout << "speed: " << speed << ", radius: " << radius << std::endl;

  velocity_mutex.unlock();
  return;
}
```

```cpp
bias(0.485), // トレッド幅[m]
wheel_radius(0.205), // 車輪半径[m]
```

| linear [m/s] | angular [rad/s] | speed [mm/s] | radius [mm] |
|------------- | --------------- | ------------ | ----------- |
|  0.1   |  0.45 | 209.125 | 222.222 |
|  0.2   |  0.85 | 406.125 | 235.294 |
|  0.3   |  1.35 | 643.125 | 238.806 |

## 信地回転が狂う瞬間の計算式

```cpp
if (radius > 0.0f) { // 右回り
    speed = (radius + 1000.0f * bias / 2.0f) * wz;
} else { // 左回り
    speed = (radius - 1000.0f * bias / 2.0f) * wz;
}
```

例：vx = 0.1, wz = 0.45のとき

```cpp
radius = vx * 1000.0f / wz
       = 0.1 * 1000.0f / 0.45
       = 222.222

speed = (radius + 1000.0f * bias / 2.0f) * wz
      = (222.222 + 1000.0f * 0.485 / 2.0f) * 0.45
      = 209.125
```

マイコン側での速度の計算式

```cpp
pulseL = (int16)((speed * (radius - bias / 2.0f) / radius) / 5)
pulseR = (int16)((speed * (radius + bias / 2.0f) / radius) / 5)
```

例：speed = 209.125, radius = 222.222, bias = 485のとき

```cpp
pulseL = (int16)((209.125 * (222.222 - 485 / 2.0f) / 222.222) / 5)
        = (int16)((209.125 * (222.222 - 242.5) / 222.222) / 5)
        = (int16)((209.125 * -20.278 / 222.222) / 5)
        = (int16)((-4241.5 / 222.222) / 5)
        = (int16)(-19.1 / 5)
        = -3

pulseR = (int16)((209.125 * (222.222 + 485 / 2.0f) / 222.222) / 5)
        = (int16)((209.125 * (222.222 + 242.5) / 222.222) / 5)
        = (int16)((209.125 * 464.722 / 222.222) / 5)
        = (int16)(971.5 / 5)
        = 194
```

ToDo:
計算している箇所をSerialPrintで出力して確認する

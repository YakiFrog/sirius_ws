#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
import math


class RotationDetectionNode(Node):
    def __init__(self):
        super().__init__('rotation_detection_node')

        # TF2バッファとリスナーの設定
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 回転前の姿勢（初期値）
        self.initial_yaw = None
        self.target_frame = 'base_footprint'
        self.source_frame = 'odom'

        # 検知有効化フラグ
        self.enabled = False

        # トピックの購読
        self.change_subscriber = self.create_subscription(
            Bool, 'change', self.change_callback, 10
        )

        # タイマーで姿勢の監視
        self.timer = self.create_timer(0.1, self.check_rotation)

    def change_callback(self, msg):
        """
        'change'トピックのコールバック関数
        """
        if msg.data:
            self.enabled = True
            self.initial_yaw = None  # 初期Yaw角をリセット
            self.get_logger().info('90度検知が有効になりました。')
        else:
            self.enabled = False
            self.get_logger().info('90度検知が無効になりました。')

    def check_rotation(self):
        """
        回転の監視
        """
        if not self.enabled:
            # 検知が無効化されている場合は何もしない
            return

        try:
            # TF情報を取得
            transform = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, rclpy.time.Time()
            )

            # クォータニオンをYaw角に変換
            yaw = self.quaternion_to_yaw(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )

            # 現在のYaw角を表示
            self.get_logger().info(f'現在のYaw角: {math.degrees(yaw):.2f}度')

            if self.initial_yaw is None:
                # 初期のYaw角を保存
                self.initial_yaw = yaw
                self.get_logger().info(f'初期Yaw角を設定: {math.degrees(yaw):.2f}度')
                return

            # 現在のYaw角との差分を計算
            yaw_difference = abs(yaw - self.initial_yaw)

            # 差分を-π～πの範囲に正規化
            yaw_difference = (yaw_difference + math.pi) % (2 * math.pi) - math.pi

            # 回転を検知
            if abs(yaw_difference) >= math.pi / 2:
                self.get_logger().info(
                    f'90度回転を検知しました！ 現在のYaw角: {math.degrees(yaw):.2f}度'
                )
                
                # マップサーバーを起動
                self.get_logger().info('マップサーバーを起動します。')
                self.create_subprocess_executable('ros2 launch nav2_map_server map_server --ros-args -p yaml_filename:=/home/sirius24/sirius_ws/map_waypoints/map/sakamoto2.yaml')
                
                # configure, activateをする
                self.get_logger().info('configure, activateを実行します。')
                self.create_subprocess_executable('ros2 lifecycle set /map_server configure')
                self.create_subprocess_executable('ros2 lifecycle set /map_server activate')

                # launchファイルの起動
                self.get_logger().info('マップ切り替え用のlaunchファイルを起動します。')
                self.create_subprocess_executable('ros2 launch sirius_navigation all_in_one_nav2.launch.py move_goal_file:=second.yaml')
                
                # プログラム終了
                self.get_logger().info('90度回転検知後、プログラムを終了します。')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().warning(f'TF取得に失敗: {e}')

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """
        クォータニオンからYaw角（Z軸回転）を計算
        """
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = RotationDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

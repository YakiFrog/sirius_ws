#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import yaml
import math
import argparse
from dataclasses import dataclass
from typing import List
from math import sin, cos
from std_msgs.msg import Bool

@dataclass
class Waypoint:
    number: int
    x: float
    y: float
    angle_radians: float
    rotate: float = 0.0
    stop: bool = False
    
class FollowWaypointsClient(Node):
    def __init__(self, start_count=1):
        super().__init__('follow_waypoints_client')
        
        # FollowWaypointsアクションクライアント
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # stopトピック用のパブリッシャー
        self.stop_publisher = self.create_publisher(Bool, '/stop', 10)
        
        # TF2リスナー
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        
        # ウェイポイントのロード
        file_path = "/home/sirius24/sirius_ws/map_waypoints/waypoints/waypoints.yaml"
        self.waypoints = self.load_waypoints(file_path)
        
        # 開始インデックス
        self.start_count = start_count - 1  # 0-indexedに変換
        
        # 現在処理中のウェイポイントインデックス
        self.current_waypoint_index = None
        self.previous_waypoint_index = None
        
        # 現在位置
        self.current_position = None
        
        # ログ出力制御用カウンタ
        self.position_check_count = 0
        
        # アクションサーバーの待機
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for FollowWaypoints action server...")
        
        # 位置取得タイマー（2秒周期に変更 - move_goal.pyと同じ）
        self.timer = self.create_timer(2.0, self.get_position)
        
    def load_waypoints(self, file_path: str) -> List[Waypoint]:
        """YAMLファイルからウェイポイントをロード"""
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        return [Waypoint(
            number=wp['number'],
            x=wp['x'],
            y=wp['y'],
            angle_radians=wp['angle_radians'],
            rotate=wp.get('rotate', 0.0),
            stop=wp.get('stop', False)
        ) for wp in data['waypoints']]
        
    def euler_to_quaternion(self, yaw):
        """オイラー角（Yaw）からQuaternionに変換"""
        return [
            0.0,  # x
            0.0,  # y
            sin(yaw / 2.0),  # z
            cos(yaw / 2.0)   # w
        ]
        
    def send_all_waypoints(self):
        """すべてのウェイポイントをまとめて送信"""
        if self.start_count >= len(self.waypoints):
            self.get_logger().error(f"Start index {self.start_count + 1} is out of range (max: {len(self.waypoints)})")
            return
        
        # ゴールメッセージの作成
        goal_msg = FollowWaypoints.Goal()
        
        # start_count以降のウェイポイントをすべて追加
        for wp in self.waypoints[self.start_count:]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(wp.x)
            pose.pose.position.y = float(wp.y)
            pose.pose.position.z = 0.0
            
            # 四元数の計算
            quat = self.euler_to_quaternion(float(wp.angle_radians))
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            goal_msg.poses.append(pose)
        
        self.get_logger().info(f"Sending {len(goal_msg.poses)} waypoints (from waypoint {self.start_count + 1})...")
        
        # ゴールを送信
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """ゴール受理時のコールバック"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return
        
        self.get_logger().info("Goal accepted!")
        
        # 結果を受け取る
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """フィードバックコールバック - 現在のウェイポイントインデックスを取得"""
        feedback = feedback_msg.feedback
        new_waypoint_index = feedback.current_waypoint
        
        # ウェイポイントが変わったことを検知（次のウェイポイントへ進んだ）
        if self.current_waypoint_index is not None and new_waypoint_index != self.current_waypoint_index:
            # 前のウェイポイントに到達したので、その stop 属性をチェック
            previous_actual_index = self.start_count + self.current_waypoint_index
            if previous_actual_index < len(self.waypoints):
                previous_wp = self.waypoints[previous_actual_index]
                self.get_logger().info(f"✅ Reached waypoint {previous_wp.number}! Checking stop command...")
                
                # stop 属性に基づいて停止/再開コマンドを送信
                if hasattr(previous_wp, 'stop'):
                    if previous_wp.stop:
                        self.publish_stop_command(True)  # 停止コマンド
                    else:
                        self.publish_stop_command(False)  # 再開コマンド
            
            # 現在のインデックスを更新
            self.current_waypoint_index = new_waypoint_index
            
            # 新しいウェイポイントへの移動開始を表示（変更時のみ）
            actual_waypoint_index = self.start_count + self.current_waypoint_index
            if actual_waypoint_index < len(self.waypoints):
                wp = self.waypoints[actual_waypoint_index]
                self.get_logger().info(
                    f"🎯 Now navigating to waypoint {wp.number}"
                )
        elif self.current_waypoint_index is None:
            # 初回のみ表示
            self.current_waypoint_index = new_waypoint_index
            actual_waypoint_index = self.start_count + self.current_waypoint_index
            if actual_waypoint_index < len(self.waypoints):
                wp = self.waypoints[actual_waypoint_index]
                self.get_logger().info(
                    f"🎯 Starting navigation to waypoint {wp.number}"
                )
    
    def result_callback(self, future):
        """結果受信時のコールバック"""
        result = future.result().result
        self.get_logger().info(f"Navigation completed! Missed waypoints: {result.missed_waypoints}")
    
    def publish_stop_command(self, should_stop: bool):
        """停止/再開コマンドを送信"""
        stop_msg = Bool()
        stop_msg.data = should_stop
        self.stop_publisher.publish(stop_msg)
        if should_stop:
            self.get_logger().info("Published STOP command.")
        else:
            self.get_logger().info("Published RESUME command.")
    
    def get_position(self):
        """現在位置を取得し、ウェイポイント到達判定と制御を実行"""
        try:
            # 最新のtransformを取得
            when = self.tfBuffer.get_latest_common_time('map', 'base_footprint')
            transform = self.tfBuffer.lookup_transform('map', 'base_footprint', when)
            translation = transform.transform.translation
            self.current_position = [translation.x, translation.y]
            
            # カウンタを増やす
            self.position_check_count += 1
            
            # 現在のウェイポイントインデックスがある場合
            if self.current_waypoint_index is not None:
                # 実際のウェイポイントインデックスに変換
                actual_waypoint_index = self.start_count + self.current_waypoint_index
                
                # インデックスが有効範囲内かチェック
                if actual_waypoint_index < len(self.waypoints):
                    current_wp = self.waypoints[actual_waypoint_index]
                    
                    # 目標との距離を計算（デバッグ表示のみ）
                    x_distance = current_wp.x - self.current_position[0]
                    y_distance = current_wp.y - self.current_position[1]
                    distance = math.sqrt(x_distance**2 + y_distance**2)
                    
                    # 距離をログ出力（10秒に1回 = 5回に1回）
                    if self.position_check_count % 5 == 0:
                        self.get_logger().info(
                            f"📏 Distance to waypoint {current_wp.number}: {distance:.2f}m"
                        )
                        
        except LookupException:
            self.get_logger().warn("Transform lookup failed. Retrying...")
        except Exception as e:
            self.get_logger().warn(f"Transform error: {str(e)}")
            
def main(args=None):
    parser = argparse.ArgumentParser(description='Follow all waypoints using FollowWaypoints action.')
    parser.add_argument('--count', type=int, default=1, help='Starting waypoint index (default: 1)')
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    node = FollowWaypointsClient(start_count=parsed_args.count)
    node.send_all_waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

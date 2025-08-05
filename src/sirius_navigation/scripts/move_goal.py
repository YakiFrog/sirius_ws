#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped
import yaml
import math
import argparse
from dataclasses import dataclass
from typing import List
from math import sin, cos, pi

@dataclass
class Waypoint:
    number: int
    x: float
    y: float
    angle_radians: float
    rotate: float = 0.0
    
class Nav2GoalClient(Node):
    def __init__(self, count = 1):
        super().__init__('nav2_goal_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        while not self._action_client.wait_for_server(timeout_sec = 1.0):
            self.get_logger().info("Waiting for action server...")
        
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        
        file_path = "/home/sirius24/sirius_ws/map_waypoints/waypoints/waypoints.yaml"
        self.waypoints = self.load_waypoints(file_path)
        self.count = count - 1
        self.loop_count = 0
        self.distance = float('inf')
        self.positions_list = []
        self.timer = self.create_timer(1.0, self.get_position)
        
    def load_waypoints(self, file_path: str) -> List[Waypoint]:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        return [Waypoint(
            number = wp['number'],
            x = wp['x'],
            y = wp['y'],
            angle_radians = wp['angle_radians'],
            rotate = wp.get('rotate', 0.0) # キーが存在しない場合は0.0を返す
        ) for wp in data['waypoints']]
        
    def euler_to_quaternion(self, yaw):
        """
        オイラー角（Yaw）からQuaternionに変換
        """
        return [
            0.0,  # x
            0.0,  # y
            sin(yaw / 2.0),  # z
            cos(yaw / 2.0)   # w
        ]
        
    def send_goal(self):
        if self.count >= len(self.waypoints):
            self.get_logger().info("All goals have been sent.")
            return
        
        wp = self.waypoints[self.count]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp.x)
        goal_msg.pose.pose.position.y = float(wp.y)
        goal_msg.pose.pose.position.z = 0.0
        
        # 四元数の計算
        quat = self.euler_to_quaternion(float(wp.angle_radians))
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]
        
        self.get_logger().info(f"Sending goal {wp.number}...")
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        if future.result().accepted:
            self.get_logger().info("Goal accepted...")
            self.distance = float('inf')  # 新しいゴールが受理されたので距離をリセット
        else:
            self.get_logger().info("Goal rejected...")
            
    def get_position(self):
        try:
            # 最新のtransformを取得
            when = self.tfBuffer.get_latest_common_time('map', 'base_footprint')
            transform = self.tfBuffer.lookup_transform(
                'map',
                'base_footprint',
                when
            )
            translation = transform.transform.translation
            self.position = [translation.x, translation.y]

            # waypointsとpositions_listの長さをチェック
            if self.count < len(self.waypoints):
                current_wp = self.waypoints[self.count]
                x_goal = current_wp.x
                y_goal = current_wp.y
                x_distance = x_goal - self.position[0]
                y_distance = y_goal - self.position[1]
                self.distance = math.sqrt(x_distance**2 + y_distance**2)
                self.get_logger().info(f"Current distance to goal: {self.distance}")

                if self.distance < 0.7:
                    self.get_logger().info("Goal reached! Sending next goal...")
                    self.count += 1
                    self.send_goal()
                
                elif self.loop_count % 5 == 0:
                    self.send_goal()
                
                self.loop_count += 1

        except LookupException:
            self.get_logger().warn("Transform lookup failed. Retrying...")
        except Exception as e:
            self.get_logger().warn(f"Transform error: {str(e)}")
            
        # タイマーの周期を2秒に変更
        self.timer.timer_period_ns = 2000000000  # 2秒
            
def main(args = None):
    parser = argparse.ArgumentParser(description='Set the starting waypoint index.')
    parser.add_argument('--count', type = int, default = 1, help = 'Starting waypoint index (default: 1)')
    parsed_args = parser.parse_args()
    
    rclpy.init(args = args)
    node = Nav2GoalClient(count = parsed_args.count)
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
import yaml
import math
import argparse
from dataclasses import dataclass
from typing import List

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
        
        self.wayponts = self.load_waypoints()
        self.count = count - 1
        self.loop_count = 0
        self.distance = float('inf')
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
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(wp.angle_radians / 2)
        goal_msg.pose.pose.orientation.w = math.cos(wp.angle_radians / 2)
        
        self.get_logger().info(f"Sending goal {wp.number}...")
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        
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
        
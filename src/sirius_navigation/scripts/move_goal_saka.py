#!/usr/bin env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
import yaml
import math 
import argparse
import time

class Nav2GoalClient(Node):
    def __init__(self, yaml_file = "move_goal.yaml"):
        super().__init__('nav2_goal_client')
        yaml_file_path = "/home/sirius24/sirius_ws/src/sirius_navigation/params/{}".format(yaml_file)
        
        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
        
        count = yaml_data["count"]
        self.stop = yaml_data["stop"]
        self.detect = yaml_data["detect"]
        self.change = yaml_data["change"]
        self.waypoint_file = yaml_data["waypoint_file"]
        
        self.action_client_ = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("起動params名：" + yaml_file_path)
        
        if self.stop:
            self.stop_list = yaml_data["stop_list"]
        else:
            self.stop_list = []
        
        if self.change:
            self.change_publisher = self.create_publisher(Bool, 'change', 10)
            
        if self.detect:
            self.detect_publisher = self.create_publisher(Bool, 'detect_person', 10)
            self.detect_list = yaml_data["detect_list"]
        else:
            self.detect_list = []
            
        while not self.action_client_.wait_for_server(timeout_sec = 1.0):
            self.get_logger().info("Waiting for action server...")
            
            self.tfBuffer = Buffer()
            self.listener = TransformListener(self.tfBuffer, self)
            
        with open(self.waypoint_file, 'r', encoding='utf-8') as f:
            self.positions_list = yaml.safe_load(f)['points']
            
        self.count = count - 1
        self.loop_count = 0
        self.distance = float('inf')
        
        self.timer = self.create_timer(1.0, self.get_position)
        
    def send_goal(self):
        if self.count >= len(self.positions_list):
            self.get_logger().info("All goals have been sent.")
            
            if self.change:
                pub_msg = Bool()
                pub_msg.data = True
                self.change_publisher.publish(pub_msg)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.y = float(self.positions_list[self.count][1])
        goal_msg.pose.pose.position.z = float(self.positions_list[self.count][2])
        goal_msg.pose.pose.orientation.x = float(self.positions_list[self.count][3])
        goal_msg.pose.pose.orientation.y = float(self.positions_list[self.count][4])
        goal_msg.pose.pose.orientation.z = float(self.positions_list[self.count][5])
        goal_msg.pose.pose.orientation.w = float(self.positions_list[self.count][6])
        
        self.get_logger().info("Sending goal points...")
        self.action_client_.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        self.get_logger().info("way point number: {}".format(self.count))
        
    def goal_response_callback(self, future):
        if future.result().accepted:
            self.get_logger().info("Goal accepted...")
            self.distance = float('inf')
        else:
            self.get_logger().info("Goal rejected...")
            
    def get_position(self):
        try:
            transform = self.tfBuffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            translation = transform.transform.translation
            self.position = [translation.x, translation.y]
            
            if self.count < len(self.positions_list):
                x_goal = self.positions_list[self.count][0]
                y_goal = self.positions_list[self.count][1]
                x_distance = x_goal - self.position[0]
                y_distance = y_goal - self.position[1]
                self.distance = math.sqrt(x_distance**2 + y_distance**2)
                self.get_logger().info(f"Current distance to goal: {self.distance}")
                
                if self.distance < 1.2:
                    self.get_logger().info("Goal reached! Sending next goal...")
                    if self.stop and ((self.count + 1) in self.stop_list):
                        self.get_logger().info("Stop position!!!")
                        time.sleep(self.stop)
                    
                    if self.detect and ((self.count + 1) in self.detect_list):
                        self.get_logger().info("Detect position!!!")
                        self.pub_msg = Bool()
                        self.pub_msg.data = True
                        self.detect_publisher.publish(self.pub_msg)
                        time.sleep(self.detect)
                    
                    self.count += 1
                    self.send_goal()
                    
                elif self.loop_count % 5 == 0:
                    self.send_goal()
                    
                self.loop_count += 1
                
        except Exception as e:
            self.get_logger().warn(f"Transform error: {e}")
            
def main(args=None):
    parser = argparse.ArgumentParser(description="Set the yaml file name.")
    parser.add_argument('--yaml_file', type=str, default="move_goal.yaml", help="Yaml file name (default: move_goal.yaml)")
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    nav2_goal_client = Nav2GoalClient(yaml_file=parsed_args.yaml_file)
    nav2_goal_client.send_goal()
    rclpy.spin(nav2_goal_client)
    nav2_goal_client.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
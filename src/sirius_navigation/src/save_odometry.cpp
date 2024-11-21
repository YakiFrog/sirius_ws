#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GetOdometry : public rclcpp::Node
{
public:
    GetOdometry() : Node("save_odometry"), yaml_file_path_("/home/sirius/sirius_ws/src/sirius_navigation/config/map.yaml")
    {
        current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GetOdometry::callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(30), std::bind(&GetOdometry::save_to_yaml, this));

        if (!current_pose_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create subscription to /odom");
        }
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr data)
    {
        std::vector<double> point = {
            data->pose.pose.position.x,
            data->pose.pose.position.y,
            data->pose.pose.position.z,
            data->pose.pose.orientation.x,
            data->pose.pose.orientation.y,
            data->pose.pose.orientation.z,
            data->pose.pose.orientation.w
        };

        // 重複チェック
        if (true_waypoints_list.empty() || point != true_waypoints_list.back()) {
            true_waypoints_list.push_back(point);
            RCLCPP_INFO(this->get_logger(), "[%f, %f, %f, %f, %f, %f, %f]",
                         point[0], point[1], point[2], point[3], point[4], point[5], point[6]);
        } else {
            RCLCPP_INFO(this->get_logger(), "Duplicate data received; not saving.");
        }
    }

    void save_to_yaml()
    {
        if (true_waypoints_list.empty()) {
            RCLCPP_INFO(this->get_logger(), "No waypoints to save.");
            return;
        }

        YAML::Node waypoint_yaml;
        waypoint_yaml["points"] = YAML::Node(YAML::NodeType::Sequence);

        for (const auto& point : true_waypoints_list) {
            YAML::Node yaml_point;
            for (const auto& val : point) {
                yaml_point.push_back(val);
            }
            waypoint_yaml["points"].push_back(yaml_point);
        }

        std::ofstream file(yaml_file_path_);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing");
            return;
        }

        file << waypoint_yaml;
        RCLCPP_INFO(this->get_logger(), "Waypoints saved to YAML file: %s", yaml_file_path_.c_str());
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> true_waypoints_list;
    std::string yaml_file_path_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetOdometry>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


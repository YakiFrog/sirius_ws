#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GetOdometry : public rclcpp::Node
{
    public:
        explicit GetOdometry() : Node("get_odometry")
        {
            current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&GetOdometry::callback, this, std::placeholders::_1));
        }
    
    private:
        void callback(const nav_msgs::msg::Odometry::SharedPtr data)
        {
            RCLCPP_INFO(get_logger(), "x :  '%f\n'", data->pose.pose.position.x);
            RCLCPP_INFO(get_logger(), "y :  '%f\n'", data->pose.pose.position.y);

        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GetOdometry>());
    rclcpp::shutdown();
    return 0;
}
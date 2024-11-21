#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <math.h>

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class MoveGoal : public rclcpp::Node
{
    public:
        explicit MoveGoal() : Node("move_goal"), count_(0), distance_(0.0)
        {
            goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
            current_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MoveGoal::callback, this, std::placeholders::_1));
            // this->declare_parameter("yaml_file_");
            // this->get_parameter("yaml_file_", yaml_file_);
            this->declare_parameter("goal_tolerance_", 2.0);
            this->get_parameter("goal_tolerance_", goal_tolerance_);
            file_path_ = "/home/sirius/sirius/sirius_ws/src/sirius_navigation/config/atc_map1.yaml";
            node_ = YAML::LoadFile(file_path_);
            goal_points_ = node_["points"].as<std::vector<std::vector<double>>>();
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        }

        struct Vector2D
        {
            double x_;
            double y_;
        };

    private:
        void callback(const nav_msgs::msg::Odometry::SharedPtr data)
        {
            auto goal_stamped_ = geometry_msgs::msg::PoseStamped();
            vector2d.x_ = data->pose.pose.position.x;
            vector2d.y_ = data->pose.pose.position.y;
            distance_ = get_disctance(goal_points_[count_][0], goal_points_[count_][1], vector2d);
            RCLCPP_INFO(get_logger(), "goal point : %f, %f", goal_points_[count_][0], goal_points_[count_][1]);
            RCLCPP_INFO(get_logger(), "%f\n", distance_);
            // geometry_msgs::msg::TransformStamped transformStamped;
            // transformStamped = tf_buffer_->lookupTransform("base_footprint", "map", tf2::TimePointZero);
            // RCLCPP_INFO(get_logger(), "transform_x:%f", transformStamped.transform.translation.x);
            // RCLCPP_INFO(get_logger(), "transform_y:%f", transformStamped.transform.translation.y);
            if(distance_ > goal_tolerance_){
		        //rad = (goal_points_[count_][2] / 180)*M_PI;
                goal_stamped_.header.stamp = get_clock()->now();
                goal_stamped_.header.frame_id = "map";
                goal_stamped_.pose.position.x = goal_points_[count_][0];
                goal_stamped_.pose.position.y = goal_points_[count_][1];
                goal_stamped_.pose.position.z = goal_points_[count_][2];
		        //goal_stamped_.pose.position.z = 0.0;
                goal_stamped_.pose.orientation.x = goal_points_[count_][3];
                //goal_stamped_.pose.orientation.x = 0.0;
		        goal_stamped_.pose.orientation.y = goal_points_[count_][4];
                //goal_stamped_.pose.orientation.y = 0.0;
	       	    goal_stamped_.pose.orientation.z = goal_points_[count_][5];
                //goal_stamped_.pose.orientation.z = sin(rad*0.5);
		        goal_stamped_.pose.orientation.w = goal_points_[count_][6];
                //goal_stamped_.pose.orientation.w = cos(rad*0.5); 
		        goal_publisher_->publish(goal_stamped_);
            }
            else{
                RCLCPP_INFO(get_logger(), "move to next waypoint");
                count_++;
            }

        }

        double get_disctance(double x, double y, Vector2D point2d)
        {
            return std::hypot((x - point2d.x_), (y - point2d.y_));
            //return std::hypot((point2d.x_ - x), (point2d.y_ - y));

        }

        int count_;
        double distance_;

        std::string file_path_;
        double goal_tolerance_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_;
        
        std::vector<std::vector<double>> goal_points_;

        YAML::Node node_;
        bool change_waypoint_flg_ = false;

        // geometry_msgs::msg::TransformStamped transformStamped;
        tf2::TimePoint timepoint;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};


	//float rad;

        Vector2D vector2d;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveGoal>());
    rclcpp::shutdown();
    return 0;
}

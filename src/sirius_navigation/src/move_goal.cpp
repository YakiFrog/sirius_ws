#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"
#include "yaml-cpp/yaml.h"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public rclcpp::Node
{
  public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_current_pose_;
  explicit Nav2Client(size_t initial_count = 0): Node("nav2_send_goal"), count_(initial_count),renew_(0){
    //アクション Client の作成
    publisher_ = this->create_publisher<std_msgs::msg::String>("waypoint_count", 10);
    publisher_current_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose_info", 10);
    this->client_ptr_  = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    file_path_ = "/home/sirius/sirius_ws/src/sirius_navigation/config/map.yaml"; 
    node_ = YAML::LoadFile(file_path_);
    goal_points_ = node_["points"].as<std::vector<std::vector<double>>>();
    //sendGoal();
    setfeedback();
  }

  void setfeedback(){
    send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Nav2Client::resultCallback, this, _1);
  }

  void sendGoal(void) {
    RCLCPP_INFO(get_logger(), "SigeSige");
    while (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    if (count_ < goal_points_.size()) {
      RCLCPP_INFO(get_logger(), "Sigemura");
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.stamp = this->now();
      goal_msg.pose.header.frame_id = "map";

      goal_msg.pose.pose.position.x = goal_points_[count_][0];
      goal_msg.pose.pose.position.y = goal_points_[count_][1];
      goal_msg.pose.pose.position.z = goal_points_[count_][2];
      goal_msg.pose.pose.orientation.x = goal_points_[count_][3];
      goal_msg.pose.pose.orientation.y = goal_points_[count_][4];
      goal_msg.pose.pose.orientation.z = goal_points_[count_][5];
      goal_msg.pose.pose.orientation.w = goal_points_[count_][6];

      // Goal をサーバーに送信
      client_ptr_->async_send_goal(goal_msg, send_goal_options);
      RCLCPP_INFO(get_logger(), "Sent goal to waypoint %ld", count_);
    } 
    else {
      RCLCPP_INFO(get_logger(), "All waypoints reached.");
    }
  }

  //feedback
  void feedbackCallback(GoalHandleNavigateToPose::SharedPtr goal_handle, const std::shared_ptr<const NavigateToPose::Feedback> feedback){
    auto pose_message = geometry_msgs::msg::PoseStamped();
    pose_message.header = feedback->current_pose.header;
    pose_message.pose = feedback->current_pose.pose;
    publisher_current_pose_->publish(pose_message);
    
    if (feedback->distance_remaining > 1.0) {
      renew_ = 1;
    }
    
    if (feedback->distance_remaining < 1.0 && renew_ == 1) {
      RCLCPP_INFO(get_logger(), "Go to next position!!!");
      count_++;
        
      if (count_ < goal_points_.size()){
        RCLCPP_INFO(get_logger(), "Send Goal!!!");
        auto message = std_msgs::msg::String();
        message.data = std::to_string(count_);
        publisher_->publish(message);
        renew_ = 0;

        // 次の目標を送信
        sendGoal();
      } 
      else {
        RCLCPP_INFO(get_logger(), "All waypoints reached.");
      }
    }
  }

  //result
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!");
        count_++; // 成功した場合はカウントをインクリメント
        sendGoal(); // 次の目標を送信
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        break;

      default:
          RCLCPP_ERROR(get_logger(), "Unknown result code");
          break;
    }
  }


  size_t count_;
  int renew_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string file_path_;
  std::vector<std::vector<double>> goal_points_;
  YAML::Node node_;

};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  // デフォルト値を設定
  size_t initial_count = 0;

  // コマンドライン引数から初期値を取得する
  if (argc > 1) {
    try {
      initial_count = std::stoul(argv[1]);
    } catch (const std::exception &e) {
      std::cerr << "Invalid argument for count: " << e.what() << std::endl;
      return 1;
    }
  }

  auto node = std::make_shared<Nav2Client>(initial_count); // count_ の初期値を設定
  node->sendGoal();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
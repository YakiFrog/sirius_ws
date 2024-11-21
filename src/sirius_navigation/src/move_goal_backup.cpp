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
    // アクションが提供されているまでに待つ
    while (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    RCLCPP_INFO(get_logger(), "count_ = %ld", count_);


    //アクション　Goalの作成
    //std::this_thread::sleep_for(std::chrono::seconds(3));
    auto goal_msg = NavigateToPose::Goal();
    if(count_ < goal_points_ .size()){
      //auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.stamp = this->now();
      goal_msg.pose.header.frame_id = "map";

      goal_msg.pose.pose.position.x = goal_points_[count_][0];
      goal_msg.pose.pose.position.y = goal_points_[count_][1];
      goal_msg.pose.pose.position.z = goal_points_[count_][2];
      goal_msg.pose.pose.orientation.x = goal_points_[count_][3];
      goal_msg.pose.pose.orientation.y = goal_points_[count_][4];
      goal_msg.pose.pose.orientation.z = goal_points_[count_][5];
      goal_msg.pose.pose.orientation.w = goal_points_[count_][6];
      //count_ ++;
    }
    RCLCPP_INFO(get_logger(), "messege");
    //進捗状況を表示するFeedbackコールバックを設定
    //auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    //send_goal_options.feedback_callback = std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
    //send_goal_options.result_callback = std::bind(&Nav2Client::resultCallback, this, _1);
    //Goal をサーバーに送信
    std::this_thread::sleep_for(std::chrono::seconds(3));
    //RCLCPP_INFO(get_logger(), "messege3");
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(get_logger(), "sendgoal");
  }
  //feedback
  void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback){
    //RCLCPP_INFO(get_logger(), "current_pose = %f", feedback-> current_pose.pose.position.x);
    //RCLCPP_INFO(get_logger(), "current_pose = %f", feedback-> current_pose.pose.position.y);
    //RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);

    auto pose_message = geometry_msgs::msg::PoseStamped();
    pose_message.header = feedback->current_pose.header;
    pose_message.pose = feedback->current_pose.pose;
    publisher_current_pose_->publish(pose_message);
        
    if (feedback->distance_remaining > 0.5){
		  renew_=1;
	  }
    if (feedback->distance_remaining < 0.5 && renew_ == 1){
      RCLCPP_INFO(get_logger(), "Go to next position!!!");
	    count_++;
      auto message = std_msgs::msg::String();
	    message.data = std::to_string(count_);
	    publisher_->publish(message);
	    renew_=0;
	    //std::this_thread::sleep_for(std::chrono::seconds(20));
	    //auto goal_msg = NavigateToPose::Goal();
	    //client_ptr_->async_send_goal(goal_msg, send_goal_options);
	    this->client_ptr_  = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
	    //client_ptr_->async_send_goal(goal_msg, send_goal_options);
	   // std::this_thread::sleep_for(std::chrono::seconds(30));
      sendGoal();
	    setfeedback();
	  }

  }
  //result
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
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
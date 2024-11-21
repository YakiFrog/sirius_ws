#include <iostream>
#include <fstream>
#include <vector> 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;
class ClickedPointSubscriber : public rclcpp::Node
{
public:
    ClickedPointSubscriber() : Node("clicked_point_subscriber")
    {
        // クリックされたポイントのトピックのsubscriberを作成
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/waypoints", 10, std::bind(&ClickedPointSubscriber::callback,  this, _1));
    }

    ~ClickedPointSubscriber()
    {
        YAML::Node waypoint_yaml;
        //std::cout << "yosiki" << std::endl;
        // YAMLファイルにデータを書き込み

        std::ofstream file("/home/sirius/sirius_ws/src/sirius_navigation/config/open_campus.yaml");
        //----------------
        waypoint_yaml["points"] = true_waypoints_list;
        YAML::Emitter out;
        out << waypoint_yaml;

        file << "points: [" << std::endl;
	for (const auto &point : true_waypoints_list) {
		file << "[";
		for (size_t i = 0; i < point.size(); ++i) {
			file << point[i];
			if (i != point.size() - 1) {
				file << ",";
			}
		}
		file << "]," << std::endl;
	}
	file << "]" << std::endl;

        //----------------
        file.close();
        //std::cout << "naoki" << std::endl;

    }

private:
    void callback(const visualization_msgs::msg::MarkerArray::SharedPtr marker_array)
    {
        
        std::vector<std::vector<double>> points;

	    for (const auto& marker : marker_array->markers)
	    {
		double x = marker.pose.position.x;
		double y = marker.pose.position.y;
		double z = marker.pose.position.z;
		double qx = marker.pose.orientation.x;
		double qy = marker.pose.orientation.y;
		double qz = marker.pose.orientation.z;
		double qw = marker.pose.orientation.w;

        points.push_back({x, y, z, qx, qy, qz, qw});
        
		// クリックされたポイントの座標を表示
		//RCLCPP_INFO(this->get_logger(),  "Received way point: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f", x, y, z, qx, qy, qz, qw);

        
	    }

        std::cout << "[";
        for(size_t i = 0; i < points.size(); ++i){
            std::cout << "[" << points[i][0] << ", " << points[i][1] << ", " << points[i][2] << ", " 
            << points[i][3] << ", " << points[i][4] << ", " << points[i][5] << ", " 
            << points[i][6] << "]" ;
                
        }
        std::cout << "]" << std::endl;
       
       
	int len_size = points.size()-1;
        std::vector<double> waypoint_vector = points[len_size];

        //std::cout << "" <<std::endl;
        true_waypoints_list.push_back(waypoint_vector);
	/*
        for(const auto&e : true_waypoints_list){
            std::cout << "hiroki: ";
            for(const auto&masako : e){
                std::cout << masako << ", ";
            }
            std::cout << "" << std::endl;
        }
	*/
         
    }


    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    std::vector<std::vector<double>> true_waypoints_list;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClickedPointSubscriber>());
    rclcpp::shutdown();
    return 0;
}


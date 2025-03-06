#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"


#include <chrono> 
#include <functional> 
#include <memory>     
#include <string>     

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <iostream>
#include <cstdlib>

#include "std_msgs/msg/string.hpp"

// dependencies for ROS
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>



#define DELTAT(_nowtime, _thentime) ((_thentime > _nowtime) ? ((0xffffffff - _thentime) + _nowtime) : (_nowtime - _thentime))


// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG




// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information; comment out to not publish (TODO: write custom roboteq messages to support other reportable data from MC

// #define _ODOM_SENSORS

// Define following to enable service for returning covariance
// #define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

serial::Serial controller;
uint32_t millis()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

namespace Roboteq
{
Roboteq::Roboteq() : Node("roboteq_ros2_driver")
// initialize parameters and variables
{
    pub_odom_tf = this->declare_parameter("pub_odom_tf", true);
    odom_frame = this->declare_parameter("odom_frame", "odom");
    base_frame = this->declare_parameter("base_frame", "base_footprint");
    cmdvel_topic = this->declare_parameter("cmdvel_topic", "cmd_vel");
    odom_topic = this->declare_parameter("odom_topic", "odom");
    port = this->declare_parameter("port", "/dev/ttyACM0");
    baud = this->declare_parameter("baud", 115200);
    open_loop = this->declare_parameter("open_loop", true);
    wheel_circumference = this->declare_parameter("wheel_circumference", 0.877); //タイヤの円周(m)
    track_width = this->declare_parameter("track_width", 0.40); //トレッド幅(m)
    max_amps = this->declare_parameter("max_amps", 5.0);
    max_rpm = this->declare_parameter("max_rpm", 100);
    gear_ratio = this->declare_parameter("gear_ratio", 1.0); //ギア比
    pulse = this->declare_parameter("pulse", 457); //一周あたりのパルス数

    starttime = 0;
    hstimer = 0;
    mstimer = 0;
    odom_idx = 0;
    odom_encoder_toss = 5;
    odom_encoder_left = 0;
    odom_encoder_right = 0;
    odom_x = 0.0;
    odom_y = 0.0;
    odom_yaw = 0.0;
    odom_last_x = 0.0;
    odom_last_y = 0.0;
    odom_last_yaw = 0.0;
    odom_last_time = 0;
    first_time = true;
    right_rpm_command = 0.0;
    left_rpm_command = 0.0;
    linear_x = 0.0;
    angular_z = 0.0;
    
    
    //odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg = nav_msgs::msg::Odometry();

    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    controller.setPort(port);
    controller.setBaudrate(baud);
    controller.setTimeout(timeout);
    // connect to serial port
    connect();
    // configure motor controller
    cmdvel_setup();
    odom_setup();
//
//  odom publisher
//
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000);
//
// cmd_vel subscriber
//

    cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        cmdvel_topic, // topic name
        1000,         // QoS history depth
        std::bind(&Roboteq::cmdvel_callback, this, std::placeholders::_1));
    using namespace std::chrono_literals;
    // set odometry publishing loop timer at 10Hz
    timer_ = this->create_wall_timer(10ms,std::bind(&Roboteq::run, this));
    // enable modifying params at run-time
    odom_baselink_transform_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    /*    
    using namespace std::chrono_literals;
    */
    param_update_timer = this->create_wall_timer(1000ms, std::bind(&Roboteq::update_parameters, this));
    
}

void Roboteq::update_parameters()
{
    RCLCPP_INFO(this->get_logger(), "Parameters updated ...");
    this->get_parameter("pub_odom_tf", pub_odom_tf);
    this->get_parameter("odom_frame", odom_frame);
    this->get_parameter("base_frame", base_frame);
    this->get_parameter("cmdvel_topic", cmdvel_topic);
    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("port", port);
    this->get_parameter("baud", baud);
    this->get_parameter("open_loop", open_loop);
    this->get_parameter("wheel_circumference", wheel_circumference);
    this->get_parameter("track_width", track_width);
    this->get_parameter("max_amps", max_amps);
    this->get_parameter("max_rpm", max_rpm);
    this->get_parameter("gear_ratio", gear_ratio);
    this->get_parameter("pulse", pulse);
}

void Roboteq::connect() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening serial port on " << port << " at " << baud << "...");
    
    while (true) {
        try {
            controller.open();
            if (controller.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
                return; 
            }
        } catch (serial::IOException &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "serial::IOException: " << e.what());
        }
        
        RCLCPP_WARN(this->get_logger(), "Failed to open serial port, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 1秒待機して再試行
    }
}

void Roboteq::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg) // const???
{
    
    //std::cout << motor_param << std::endl;
    //RCLCPP_INFO(this->get_logger(),"motor_param:%f",motor_param);

    // wheel speed (m/s)
    // 右の車輪の回転速度 = 直進速度 + トレッド幅 * 回転速度 / 2 
    float right_speed = (twist_msg->linear.x + track_width * twist_msg->angular.z / 2.0) * -1;
    // 左の車輪の回転速度 = 直進速度 - トレッド幅 * 回転速度 / 2
    float left_speed = (twist_msg->linear.x - track_width * twist_msg->angular.z / 2.0) * -1;

    linear_x = twist_msg->linear.x;
    angular_z = twist_msg->angular.z;

    //超信地旋回
    if (std::abs(linear_x) < 0.05)
    {
        if(right_speed > 0)
        {
            right_speed += 0.05;
            left_speed -= 0.05;
        }
        else if(right_speed < 0)
        {
            right_speed -= 0.05;
            left_speed += 0.05;
        }
    }

    //信地旋回
    else if ((std::abs(right_speed) < 0.05) && (std::abs(left_speed) < 0.05))
    {
        if (std::abs(right_speed) > std::abs(left_speed))
        {
            if (right_speed >= 0.01)
            {
                right_speed = 0.05;
                left_speed = 0.0;
                RCLCPP_INFO(this->get_logger(),"sigesige");
                
            }
            else if(right_speed <= 0.01)
            {
                right_speed = -0.05;
                left_speed = 0.0;
            }
        }
        else if (std::abs(right_speed) < std::abs(left_speed))
        {
            if (left_speed >= 0.01)
            {
                left_speed = 0.05;
                right_speed = 0.0;
            }
            else if (left_speed <= 0.01)
            {
                left_speed = 0.05;
                right_speed = 0.0;
            }
        }
    }



    std::stringstream left_cmd;
    std::stringstream right_cmd;
    

    if (open_loop)
    {
        // motor power (scale 0-1000)
        int32_t right_power = right_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;
        int32_t left_power = left_speed / wheel_circumference * 60.0 / max_rpm * 1000.0;
        /*
        // set minimum to overcome friction if cmd_vel too low
        if (right_power < 150 && left_power > 0){
            right_power = 150
        }
        if (left_power < 150 && left_power > 0){
            left_power = 150
        }
        */
        
        right_cmd << "!G 1 " << right_power << "\r";
        left_cmd << "!G 2 " << left_power << "\r";
    }
    else
    {
        // motor speed (rpm)
        int32_t right_rpm = right_speed / wheel_circumference * 60.0;
        int32_t left_rpm = left_speed / wheel_circumference * 60.0;

        right_rpm_command = right_speed / wheel_circumference * 60.0;
        left_rpm_command = left_speed / wheel_circumference * 60.0;

        right_cmd << "!S 1 " << right_rpm << "\r";
        left_cmd << "!S 2 " << left_rpm << "\r";
    }
//write cmd to motor controller
#ifndef _CMDVEL_FORCE_RUN
    controller.write(right_cmd.str());
    controller.write(left_cmd.str());
    controller.flush();
#endif
}
void Roboteq::cmdvel_setup()
{
    RCLCPP_INFO(this->get_logger(), "configuring motor controller...");

    // stop motors
    controller.write("!G 1 0\r");
    controller.write("!G 2 0\r");
    controller.write("!S 1 0\r");
    controller.write("!S 2 0\r");
    controller.flush();

    // disable echo
    controller.write("^ECHOF 1\r");
    controller.flush();

    // enable watchdog timer (1000 ms)
    controller.write("^RWD 1000\r");

    // set motor operating mode (1 for closed-loop speed)
    if (open_loop)
    {
        // open-loop speed mode
        controller.write("^MMOD 1 0\r");
        controller.write("^MMOD 2 0\r");
    }
    else
    {
        // closed-loop speed mode
        controller.write("^MMOD 1 1\r");
        controller.write("^MMOD 2 1\r");
    }

    // set motor amps limit (A * 10)
    std::stringstream right_ampcmd;
    std::stringstream left_ampcmd;
    right_ampcmd << "^ALIM 1 " << (int)(max_amps * 10) << "\r";
    left_ampcmd << "^ALIM 2 " << (int)(max_amps * 10) << "\r";
    controller.write(right_ampcmd.str());
    controller.write(left_ampcmd.str());

    // set max speed (rpm) for relative speed commands
    std::stringstream right_rpmcmd;
    std::stringstream left_rpmcmd;
    right_rpmcmd << "^MXRPM 1 " << max_rpm << "\r";
    left_rpmcmd << "^MXRPM 2 " << max_rpm << "\r";
    controller.write(right_rpmcmd.str());
    controller.write(left_rpmcmd.str());

    // set max acceleration rate (2000 rpm/s * 10)
    controller.write("^MAC 1 20000\r");
    controller.write("^MAC 2 20000\r");

    // set max deceleration rate (2000 rpm/s * 10)
    controller.write("^MDEC 1 20000\r");
    controller.write("^MDEC 2 20000\r");

    // set PID parameters (gain * 10)
    controller.write("^KP 1 10\r");
    controller.write("^KP 2 10\r");
    controller.write("^KI 1 80\r");
    controller.write("^KI 2 80\r");
    controller.write("^KD 1 0\r");
    controller.write("^KD 2 0\r");

    // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
    controller.write("^EMOD 1 18\r");
    controller.write("^EMOD 2 34\r");

    // set encoder counts (ppr)
    std::stringstream right_enccmd;
    std::stringstream left_enccmd;
    right_enccmd << "^EPPR 1 " << encoder_ppr << "\r";
    left_enccmd << "^EPPR 2 " << encoder_ppr << "\r";
    controller.write(right_enccmd.str());
    controller.write(left_enccmd.str());

    controller.flush();
}

void Roboteq::cmdvel_loop()
{
}

void Roboteq::cmdvel_run()
{
#ifdef _CMDVEL_FORCE_RUN
    if (open_loop)
    {
    controller.write("!G 1 100\r");
    controller.write("!G 2 100\r");
    }
    else
    {
    std::stringstream right_cmd;
    std::stringstream left_cmd;
    right_cmd << "!S 1 " << (int)(max_rpm * 0.1) << "\r";
    left_cmd << "!S 2 " << (int)(max_rpm * 0.1) << "\r";
    controller.write(right_cmd.str());
    controller.write(left_cmd.str());
    }
    controller.flush();
#endif
}


void Roboteq::odom_setup()
{
    RCLCPP_INFO(this->get_logger(),"setting up odom...");

    odom_baselink_transform_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    if (pub_odom_tf)
    {
        odom_msg.header.stamp = this->get_clock()->now();
    
        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id = base_frame;

        // Set up the pose covariance
        for (size_t i = 0; i < 36; i++)
        {
            odom_msg.pose.covariance[i] = 0;
            odom_msg.twist.covariance[i] = 0;
        }

        odom_msg.pose.covariance[7] = 0.001;
        odom_msg.pose.covariance[14] = 1000000;
        odom_msg.pose.covariance[21] = 1000000;
        odom_msg.pose.covariance[28] = 1000000;
        odom_msg.pose.covariance[35] = 1000;

        // Set up the twist covariance
        odom_msg.twist.covariance[0] = 0.001;
        odom_msg.twist.covariance[7] = 0.001;
        odom_msg.twist.covariance[14] = 1000000;
        odom_msg.twist.covariance[21] = 1000000;
        odom_msg.twist.covariance[28] = 1000000;
        odom_msg.twist.covariance[35] = 1000;

        // Set up the transform message: move to odom_publish
    
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_yaw);

        // start encoder streaming
        RCLCPP_INFO_STREAM(this->get_logger(),"covariance set");
        RCLCPP_INFO_STREAM(this->get_logger(),"odometry stream starting...");
        odom_stream();
    
        odom_last_time = millis();
    }
#ifdef _ODOM_SENSORS
    current_last_time = millis();
#endif
}

// Odom msg streams

void Roboteq::odom_stream()
{

#ifdef _ODOM_SENSORS
    // start encoder and current output (30 hz)
    // doubling frequency since one value is output at each cycle
    //  controller.write("# C_?CR_?BA_# 17\r");
    // start encoder, current and voltage output (30 hz)
    // tripling frequency since one value is output at each cycle
    controller.write("# C_?CB_?BA_?V_# 11\r");
#else
    // start encoder output (1000 hz)
    // Read Absolute Brush less Counter -> ?CB
    // 参照url:https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v21/file
    controller.write("# C_?CB_# 1000\r");

#endif
    controller.flush();
}

void Roboteq::odom_loop()
{
    //現在時刻の取得
    uint32_t nowtime = millis();

    // if we haven't received encoder counts in some time then restart streaming
    if (DELTAT(nowtime, odom_last_time) >= 1000)
    {
        odom_stream();
        odom_last_time = nowtime;
    }

    // read sensor data stream from motor controller
    // maybe use while loop to improve cpu usage?
    if (controller.available())
    {
        char ch = 0;
        if (controller.read((uint8_t *)&ch, 1) == 0)
            return;
        if (ch == '\r')
        {
            odom_buf[odom_idx] = 0;
            RCLCPP_INFO_STREAM(this->get_logger(), "odom_buf: " << odom_buf);
            // CB=右:左 (blush less counts)
            if (odom_buf[0] == 'C' && odom_buf[1] == 'B' && odom_buf[2] == '=')
            {
                unsigned int delim;
                for (delim = 3; delim < odom_idx; delim++)
                {
                    if (odom_encoder_toss > 0)
                    {
                        --odom_encoder_toss;
                        break;
                    }
                    if (odom_buf[delim] == ':')
                    {
                        odom_buf[delim] = 0;
                        //10進数にして代入
                        odom_encoder_right = (int32_t)strtol(odom_buf + 3, NULL, 10);
                        odom_encoder_left = (int32_t)strtol(odom_buf + delim + 1, NULL, 10);

                        //(現在のカウント - 前回のカウント) / 1回転あたりのカウント数
                        //左は正負逆でカウントされるため-1をかける
                        odom_roll_right = -1 * ((float)odom_encoder_right - odom_encoder_right_old) / pulse;
                        odom_roll_left = -1 * ((float)odom_encoder_left - odom_encoder_left_old)/ pulse;
                        RCLCPP_INFO_STREAM(this->get_logger(), "odom_roll_right: " << odom_roll_right);
                        RCLCPP_INFO_STREAM(this->get_logger(), "odom_roll_left: " << odom_roll_left);

                        /*
                        if(straight)
                        {
                            odom_roll_right = (odom_roll_right + odom_roll_left) / 2.0;
                            odom_roll_left = odom_roll_right;
                        }
                        */

                        //スタート時のみ
                        if(first_time)
                        {  
                            odom_roll_right = 0.0;
                            odom_roll_left = 0.0;
                            first_time = false;
                        }
                        
                        //現在のカウントを記憶
                        odom_encoder_right_old = (float)odom_encoder_right;
                        odom_encoder_left_old = (float)odom_encoder_left;

                        odom_publish();
                        break;
                    }
                }
            }

            odom_idx = 0;
        }
        else if (odom_idx < (sizeof(odom_buf) - 1))
        {
            odom_buf[odom_idx++] = ch;
        }
    }
}
void Roboteq::odom_publish()
{

    geometry_msgs::msg::TransformStamped tf_msg;

    // determine delta time in seconds
    uint32_t nowtime = millis();
    float dt = (float)DELTAT(nowtime, odom_last_time) / 1000.0;
    odom_last_time = nowtime;

    // determine deltas of distance and angle
    //前進距離 = (右の車輪の回転数 + 左の車輪の回転数) * タイヤの円周 / 2
    float linear = ((float)odom_roll_right + (float)odom_roll_left) * wheel_circumference / 2.0;
    //旋回角度 = (右の車輪の回転数 - 左の車輪の回転数) * タイヤの円周 / トレッド幅
    float angular = ((float)odom_roll_right - (float)odom_roll_left) * wheel_circumference / track_width;
    //RCLCPP_INFO_STREAM(this->get_logger(), "linear: " << linear);
    //RCLCPP_INFO_STREAM(this->get_logger(), "angular: " << angular);
    // Update odometry
    odom_x += linear * cos(odom_yaw);         // m
    odom_y += linear * sin(odom_yaw);         // m
    odom_yaw = NORMALIZE(odom_yaw + angular); // rad

    odom_last_x = odom_x;
    odom_last_y = odom_y;
    odom_last_yaw = odom_yaw;

    RCLCPP_INFO_STREAM(this->get_logger(), "odom_yaw " << odom_yaw);

    // convert yaw to quat;
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0, 0, odom_yaw);
    // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf2_quat);

    //update odom msg

    quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), odom_yaw));

    if(pub_odom_tf)
    {
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = odom_frame;
        tf_msg.child_frame_id = base_frame;
        tf_msg.transform.translation.x = odom_x;
        tf_msg.transform.translation.y = odom_y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = quat;
        odom_baselink_transform_->sendTransform(tf_msg);
    }

    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.pose.pose.position.x = odom_x * dt;
    odom_msg.pose.pose.position.y = odom_y * dt;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular;
    odom_pub->publish(odom_msg);
}

int Roboteq::run() {
    starttime = millis();
    hstimer = starttime;
    mstimer = starttime;
    lstimer = starttime;
        
    cmdvel_loop();
    odom_loop();
    
    // デバイスの接続状態をチェック
    if (!controller.isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Device disconnected, attempting to reconnect...");
        connect();  // 再接続を試みる
    }

    cmdvel_run();
    
    return 0;
}


Roboteq::~Roboteq()
{

    if (controller.isOpen()){
        controller.close();
    }
}

} // end of namespace

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Roboteq::Roboteq>();
    exec.add_node(node);
    exec.spin();
    printf("stop");
    rclcpp::shutdown();
    return 0;
}
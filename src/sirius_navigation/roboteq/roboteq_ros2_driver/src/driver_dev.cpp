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
#include "std_msgs/msg/bool.hpp"

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
    port = this->declare_parameter("port", "/dev/roboteq");
    baud = this->declare_parameter("baud", 115200);
    open_loop = this->declare_parameter("open_loop", true);
    wheel_circumference = this->declare_parameter("wheel_circumference", 0.877); //タイヤの円周(m)
    track_width = this->declare_parameter("track_width", 0.40); //トレッド幅(m)
    max_amps = this->declare_parameter("max_amps", 5.0);
    max_rpm = this->declare_parameter("max_rpm", 100);
    gear_ratio = this->declare_parameter("gear_ratio", 1.0); //ギア比
    pulse = this->declare_parameter("pulse", 229); //一周あたりのパルス数
    max_speed = this->declare_parameter("max_speed", 0.5); //最大速度

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
    running_ = true;
    
    odom_thread_ = std::thread(&Roboteq::odom_loop, this);

    //odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg = nav_msgs::msg::Odometry();

    serial::Timeout timeout = serial::Timeout::simpleTimeout(50); // タイムアウトを短く設定
    controller.setPort(port);
    controller.setBaudrate(baud);
    controller.setTimeout(timeout);
    // connect to serial port
    connect();
    // configure motor controller
    cmdvel_setup();
    odom_setup();
//
//  odom publisher (using SensorDataQoS for compatibility with EKF)
//
    auto qos = rclcpp::SensorDataQoS();
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, qos);
//
// cmd_vel subscriber
//

    cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        cmdvel_topic, // topic name
        1000,         // QoS history depth
        std::bind(&Roboteq::cmdvel_callback, this, std::placeholders::_1));

    stop_sub = this->create_subscription<std_msgs::msg::Bool>(
        "stop",
        10,
        std::bind(&Roboteq::bumper_callback, this, std::placeholders::_1)
    );
    
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
    this->get_parameter("max_speed", max_speed);
}

void Roboteq::connect() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening serial port on " << port << " at " << baud << "...");
    
    // 接続前に閉じておく
    if (controller.isOpen()) {
        try {
            controller.close();
        } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Error closing port: " << e.what());
        }
    }
    
    int retry_count = 0;
    const int max_retries = 5;
    
    while (retry_count < max_retries) {
        try {
            controller.open();
            if (controller.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
                
                // シリアルポートの設定を最適化
                controller.setFlowcontrol(serial::flowcontrol_none);  // フロー制御を無効化
                controller.setBytesize(serial::eightbits);  // 8ビット
                controller.setParity(serial::parity_none);  // パリティなし
                controller.setStopbits(serial::stopbits_one);  // ストップビット1
                
                return;
            }
        } catch (const serial::IOException &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Serial::IOException: " << e.what());
        } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Exception: " << e.what());
        }
        
        retry_count++;
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to open serial port, retry " << retry_count << " of " << max_retries << "...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port after maximum retries");
}

void Roboteq::bumper_callback(const std_msgs::msg::Bool::SharedPtr stop_msg)
{
    if (stop_msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "Bumper!!!");
        safe_serial_write("!EX 1\r");
    }

    else
    {
        RCLCPP_INFO(this->get_logger(), "Restart!!!");
        safe_serial_write("!MG\r");
    }
}

// シリアルポートへの安全な書き込み関数を追加
bool Roboteq::safe_serial_write(const std::string &cmd) {
    if (!controller.isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Cannot write to closed port. Attempting to reconnect...");
        try {
            connect();
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to reconnect: " << e.what());
            return false;
        }
    }
    
    try {
        controller.write(cmd);
        return true;
    } catch (const serial::IOException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Serial write failed: " << e.what());
        return false;
    } catch (const serial::PortNotOpenedException &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Port not opened: " << e.what());
        return false;
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown error during serial write: " << e.what());
        return false;
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

    //指令値が小さすぎる場合
    if ((std::abs(right_speed) < 0.10) && (std::abs(left_speed) < 0.10) && ((linear_x != 0.0) || (angular_z != 0.0)))
    {
        //ゆっくり直進する場合
        if (std::abs(angular_z) == 0.0)
        {
            if (linear_x > 0)
            {
                right_speed = -0.10;
                left_speed = -0.10;
            }
            else
            {
                right_speed = 0.10;
                left_speed = 0.10;
            }
        }

        //超信地旋回(左右のスピード差が小さい場合)
        else if (std::abs(std::abs(right_speed) - std::abs(left_speed)) < 0.01)
        {
            if (right_speed > 0)
            {
                right_speed = 0.1;
                left_speed = -0.1;
            }
            else
            {
                right_speed = -0.1;
                left_speed = 0.1;
            }
        }

        //信地旋回
        else
        {
            if (std::abs(right_speed) > std::abs(left_speed))
            {
                if (right_speed > 0)
                {
                    right_speed = 0.1;
                    left_speed = 0.0;
                }
                else
                {
                    right_speed = -0.1;
                    left_speed = 0.0;
                }
            }
            else
            {
                if (left_speed > 0.0)
                {
                    right_speed = 0.0;
                    left_speed = 0.1;
                }
                else
                {
                    right_speed = 0.0;
                    left_speed = -0.1;
                }
            }
        }
    }

    //制限速度適用処理
    if (right_speed > max_speed)
    {
        right_speed = max_speed;
    }
    if (left_speed > max_speed)
    {
        left_speed = max_speed;
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
    safe_serial_write(right_cmd.str());
    safe_serial_write(left_cmd.str());
    try {
        if (controller.isOpen()) {
            controller.flush();
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_callback flush: " << e.what());
    }
#endif
}
void Roboteq::cmdvel_setup()
{
    // stop motors
    safe_serial_write("!G 1 0\r");
    safe_serial_write("!G 2 0\r");
    safe_serial_write("!S 1 0\r");
    safe_serial_write("!S 2 0\r");
    
    if (!controller.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port not open, cannot complete setup");
        return;
    }
    
    try {
        controller.flush();

        // disable echo
        safe_serial_write("^ECHOF 1\r");
        controller.flush();

        // enable watchdog timer (1000 ms)
        safe_serial_write("^RWD 1000\r");

        // set motor operating mode (1 for closed-loop speed)
        if (open_loop)
        {
            // open-loop speed mode
            safe_serial_write("^MMOD 1 0\r");
            safe_serial_write("^MMOD 2 0\r");
        }
        else
        {
            // closed-loop speed mode
            safe_serial_write("^MMOD 1 1\r");
            safe_serial_write("^MMOD 2 1\r");
        }

        // set motor amps limit (A * 10)
        std::stringstream right_ampcmd;
        std::stringstream left_ampcmd;
        right_ampcmd << "^ALIM 1 " << (int)(max_amps * 10) << "\r";
        left_ampcmd << "^ALIM 2 " << (int)(max_amps * 10) << "\r";
        safe_serial_write(right_ampcmd.str());
        safe_serial_write(left_ampcmd.str());

        // set max speed (rpm) for relative speed commands
        std::stringstream right_rpmcmd;
        std::stringstream left_rpmcmd;
        right_rpmcmd << "^MXRPM 1 " << max_rpm << "\r";
        left_rpmcmd << "^MXRPM 2 " << max_rpm << "\r";
        safe_serial_write(right_rpmcmd.str());
        safe_serial_write(left_rpmcmd.str());

        // set max acceleration rate (2000 rpm/s * 10)
        safe_serial_write("^MAC 1 20000\r");
        safe_serial_write("^MAC 2 20000\r");

        // set max deceleration rate (2000 rpm/s * 10)
        safe_serial_write("^MDEC 1 20000\r");
        safe_serial_write("^MDEC 2 20000\r");

        // set PID parameters (gain * 10)
        safe_serial_write("^KP 1 10\r");
        safe_serial_write("^KP 2 10\r");
        safe_serial_write("^KI 1 80\r");
        safe_serial_write("^KI 2 80\r");
        safe_serial_write("^KD 1 0\r");
        safe_serial_write("^KD 2 0\r");

        // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
        safe_serial_write("^EMOD 1 18\r");
        safe_serial_write("^EMOD 2 34\r");

        // set encoder counts (ppr)
        std::stringstream right_enccmd;
        std::stringstream left_enccmd;
        right_enccmd << "^EPPR 1 " << pulse << "\r";
        left_enccmd << "^EPPR 2 " << pulse << "\r";
        safe_serial_write(right_enccmd.str());
        safe_serial_write(left_enccmd.str());

        controller.flush();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_setup: " << e.what());
    }
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
    odom_baselink_transform_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize odom message frame IDs (always needed, not just for TF)
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;

    // Set up the pose covariance (always needed for EKF)
    for (size_t i = 0; i < 36; i++)
    {
        odom_msg.pose.covariance[i] = 0;
        odom_msg.twist.covariance[i] = 0;
    }

    odom_msg.pose.covariance[0] = 0.001;   // x
    odom_msg.pose.covariance[7] = 0.001;   // y
    odom_msg.pose.covariance[14] = 1000000; // z (not used in 2D)
    odom_msg.pose.covariance[21] = 1000000; // roll (not used in 2D)
    odom_msg.pose.covariance[28] = 1000000; // pitch (not used in 2D)
    odom_msg.pose.covariance[35] = 1000;    // yaw

    // Set up the twist covariance
    odom_msg.twist.covariance[0] = 0.001;   // vx
    odom_msg.twist.covariance[7] = 0.001;   // vy
    odom_msg.twist.covariance[14] = 1000000; // vz (not used in 2D)
    odom_msg.twist.covariance[21] = 1000000; // vroll (not used in 2D)
    odom_msg.twist.covariance[28] = 1000000; // vpitch (not used in 2D)
    odom_msg.twist.covariance[35] = 1000;    // vyaw

    if (pub_odom_tf)
    {
        // Set up the transform message: move to odom_publish
    
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_yaw);

        // start encoder streaming
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
    safe_serial_write("# C_?CB_?BA_?V_# 11\r");
#else
    safe_serial_write("# C_?CB_# 50\r");  // 50ms
    
    // オプション: より小さなデータフォーマットを使用
    // 注: お使いのRoboteqコントローラがこのコマンドをサポートしているか確認してください
    // safe_serial_write("# _?CR_# 500\r");  // 生カウンタデータ (より小さなデータサイズ)
#endif
    
    try {
        if (controller.isOpen()) {
            controller.flush();
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in odom_stream flush: " << e.what());
    }
}

void Roboteq::odom_loop()
{
    const size_t buffer_size = 1024;  // バッファサイズを大きく
    uint8_t buffer[buffer_size];
    size_t bytes_read = 0;
    std::string line_buffer;
    
    // データ処理のパフォーマンス計測用
    uint32_t last_diagnostic_time = millis();
    size_t total_bytes = 0;
    int packet_count = 0;
    
    while (running_) {
        // ストリーミング設定の再送間隔を30秒に長く
        uint32_t nowtime = millis();
        if (DELTAT(nowtime, odom_last_time) >= 30000) {
            odom_stream();
            odom_last_time = nowtime;
            
            // データレート診断を表示
            float elapsed_secs = DELTAT(nowtime, last_diagnostic_time) / 1000.0f;
            if (elapsed_secs > 0 && packet_count > 0) {
                float bytes_per_sec = total_bytes / elapsed_secs;
                RCLCPP_INFO(this->get_logger(), "Odometry data rate: %.2f KB/s, packets: %d", 
                            bytes_per_sec / 1024.0f, packet_count);
                total_bytes = 0;
                packet_count = 0;
                last_diagnostic_time = nowtime;
            }
        }

        if (controller.available()) {
            try {
                // 一度に複数バイトを読み取る
                bytes_read = controller.read(buffer, buffer_size);
                total_bytes += bytes_read;
                
                // 読み取ったデータをより効率的に処理
                for (size_t i = 0; i < bytes_read; i++) {
                    char ch = buffer[i];
                    
                    if (ch == '\r') {
                        packet_count++;
                        
                        // 行の終わり - データを処理
                        if (line_buffer.length() >= 3 && 
                            line_buffer[0] == 'C' && 
                            line_buffer[1] == 'B' && 
                            line_buffer[2] == '=') {
                                
                            if (odom_encoder_toss > 0) {
                                --odom_encoder_toss;
                            } else {
                                // データを解析 - より効率的な実装
                                size_t delimiter_pos = line_buffer.find(':', 3);
                                if (delimiter_pos != std::string::npos) {
                                    try {
                                        // stoi()は例外を投げる可能性があるため、try内で実行
                                        std::string right_str = line_buffer.substr(3, delimiter_pos - 3);
                                        std::string left_str = line_buffer.substr(delimiter_pos + 1);
                                        
                                        int32_t right_val = std::stoi(right_str);
                                        int32_t left_val = std::stoi(left_str);
                                        
                                        odom_encoder_right = right_val;
                                        odom_encoder_left = left_val;

                                        float right_diff = ((float)odom_encoder_right - odom_encoder_right_old) / pulse;
                                        float left_diff = ((float)odom_encoder_left - odom_encoder_left_old) / pulse;

                                        // 値が大きすぎる場合はスキップ（エラー防止）
                                        if (std::abs(right_diff) > 100.0f || std::abs(left_diff) > 100.0f) {
                                            odom_encoder_right_old = (float)odom_encoder_right;
                                            odom_encoder_left_old = (float)odom_encoder_left;
                                            continue;
                                        }

                                        odom_roll_right = -1 * right_diff;
                                        odom_roll_left = -1 * left_diff;

                                        if (first_time) {
                                            odom_roll_right = 0.0;
                                            odom_roll_left = 0.0;
                                            first_time = false;
                                        }

                                        odom_encoder_right_old = (float)odom_encoder_right;
                                        odom_encoder_left_old = (float)odom_encoder_left;

                                    }
                                    catch (const std::exception& e) {
                                        // データ解析エラーの無視 - ログに記録しない（頻度が高すぎるため）
                                    }
                                }
                            }
                        }
                        
                        // バッファをクリア
                        line_buffer.clear();
                    } else if (line_buffer.length() < 64) {  // バッファオーバーフロー防止
                        line_buffer += ch;
                    }
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Exception reading serial data: " << e.what());
            }
        }
        odom_publish();
        odom_roll_right = 0.0;
        odom_roll_left = 0.0;

        // スリープ時間を短縮（CPUの空回しを防ぎつつも、応答性を保つ）
        std::this_thread::sleep_for(std::chrono::microseconds(100));  // 0.1 ms
    }
}

void Roboteq::odom_publish()
{
    static rclcpp::Time last_publish_time = this->get_clock()->now();
    rclcpp::Time current_time = this->get_clock()->now();
    
    // Odometryメッセージの発行頻度を制限（リソース節約のため）
    // 50Hzが一般的なナビゲーションシステムにとって十分
    if ((current_time - last_publish_time).seconds() < 0.02) {  // 50Hz
        return;
    }
    
    last_publish_time = current_time;
    
    // 以降は既存のodom_publish関数と同じ
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
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;
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
        
    try {
        cmdvel_loop();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_loop: " << e.what());
    }
    
    // デバイスの接続状態をチェック
    if (!controller.isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Device disconnected, attempting to reconnect...");
        try {
            connect();
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to reconnect: " << e.what());
        }
    }

    try {
        cmdvel_run();
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception in cmdvel_run: " << e.what());
    }
    
    return 0;
}


Roboteq::~Roboteq()
{
    running_ = false;
    if (odom_thread_.joinable()){
        odom_thread_.join();
    }

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
// zed_imu_receiver.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <mutex>
#include <memory>
#include <functional>
#include <chrono>
#include <iostream>

class ZedImuReceiver
{
public:
    // 与原有IMU相同的数据结构
    struct IMUData
    {
        float acc[3]{0, 0, 0};      // 加速度 (m/s²)
        float gyro[3]{0, 0, 0};     // 角速度 (rad/s)
        float angle[3]{0, 0, 0};    // 欧拉角 (度)
        
        // 调试输出
        std::string toString() const
        {
            char buffer[256];
            snprintf(buffer, sizeof(buffer), 
                     "Acc:[%.3f, %.3f, %.3f] Gyro:[%.3f, %.3f, %.3f] Angle:[%.3f°, %.3f°, %.3f°]", 
                     acc[0], acc[1], acc[2],
                     gyro[0], gyro[1], gyro[2],
                     angle[0], angle[1], angle[2]);
            return std::string(buffer);
        }
        
        // 获取俯仰角相关数据（常用）
        float pitchAngle() const { return angle[1]; }     // 俯仰角
        float pitchRate() const { return gyro[1]; }       // 俯仰角速度
        float yawAngle() const { return angle[2]; }       // 偏航角
        float yawRate() const { return gyro[2]; }         // 偏航角速度
    };

private:
    class ZedImuNode : public rclcpp::Node
    {
    public:
        ZedImuNode(const std::function<void(const IMUData&)>& callback = nullptr)
            : Node("zed_imu_receiver"),
              callback_(callback),
              last_update_time_(std::chrono::steady_clock::now()),
              data_valid_(false)
        {
            // 订阅IMU话题
            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data_raw",  // ZED IMU默认话题
                10,  // QoS队列深度
                std::bind(&ZedImuNode::imuCallback, this, std::placeholders::_1));
            
            RCLCPP_INFO(this->get_logger(), "ZED IMU订阅器启动，监听话题: /zed/zed_node/imu/data");
            
            // 可选：定时检查连接状态
            timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&ZedImuNode::checkConnection, this));
        }
        
        IMUData getLatestData() const
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            return latest_data_;
        }
        
        bool isDataValid() const
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_update_time_);
            return data_valid_ && elapsed.count() < 200;  // 200ms内有效
        }
        
        double getFrequency() const
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            if (update_count_ < 2) return 0.0;
            auto duration = std::chrono::duration<double>(
                last_update_time_ - first_update_time_).count();
            return (update_count_ - 1) / duration;
        }
        
    private:
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            // 解析IMU数据
            IMUData new_data;
            
            // 1. 加速度
            new_data.acc[0] = static_cast<float>(msg->linear_acceleration.x);
            new_data.acc[1] = static_cast<float>(msg->linear_acceleration.y);
            new_data.acc[2] = static_cast<float>(msg->linear_acceleration.z);
            
            // 2. 角速度
            new_data.gyro[0] = static_cast<float>(msg->angular_velocity.x);
            new_data.gyro[1] = static_cast<float>(msg->angular_velocity.y);
            new_data.gyro[2] = static_cast<float>(msg->angular_velocity.z);
            
            // 3. 从四元数转换为欧拉角
            tf2::Quaternion q(
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
                msg->orientation.w
            );
            
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
         
            
            // 弧度转角度
            new_data.angle[0] = static_cast<float>(roll * 180.0 / M_PI);    // 横滚角
            new_data.angle[1] = static_cast<float>(pitch * 180.0 / M_PI);   // 俯仰角
            new_data.angle[2] = static_cast<float>(yaw * 180.0 / M_PI);     // 偏航角

            new_data.gyro[1] = static_cast<float>(new_data.gyro[1] * 180.0 / M_PI);    // 俯仰角速度
            new_data.gyro[2] = static_cast<float>(new_data.gyro[2] * 180.0 / M_PI);     // 偏航角速度
            
            // 4. 根据你的机器人坐标系进行调整
            // 注意：ZED坐标系的Y轴可能是向下的，需要根据实际情况调整
            new_data.angle[1] = new_data.angle[1];  // 俯仰角取反
            new_data.gyro[1] = new_data.gyro[1];    // 俯仰角速度取反
            new_data.gyro[2] = new_data.gyro[2];    // 偏航角速度取反
            
            // 更新数据
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_data_ = new_data;
                last_update_time_ = std::chrono::steady_clock::now();
                data_valid_ = true;
            }
            
            // 更新统计
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                if (update_count_ == 0) {
                    first_update_time_ = last_update_time_;
                }
                update_count_++;
            }
            
            // 调用用户回调函数
            if (callback_) {
                callback_(new_data);
            }
            
            // 调试输出（控制频率）
            static int count = 0;
            if (count++ % 50 == 0) {  // 每50条消息输出一次
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "IMU更新: 俯仰角=%.2f°, 俯仰角速度=%.2f rad/s, 偏航角=%.2f°, 频率=%.1f Hz",
                    new_data.angle[1], new_data.gyro[1], new_data.angle[2],
                    getFrequency());
            }
        }
        
        void checkConnection()
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_update_time_);
                
            if (elapsed.count() > 1000) {  // 1秒没收到数据
                RCLCPP_WARN(this->get_logger(), 
                    "IMU数据超时: 最后更新 %.1f 秒前", elapsed.count() / 1000.0);
                data_valid_ = false;
            } else if (!data_valid_) {
                RCLCPP_INFO(this->get_logger(), "IMU数据已恢复");
                data_valid_ = true;
            }
        }
        
        std::function<void(const IMUData&)> callback_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        mutable std::mutex data_mutex_;
        IMUData latest_data_;
        std::chrono::steady_clock::time_point last_update_time_;
        bool data_valid_;
        
        mutable std::mutex stats_mutex_;
        std::chrono::steady_clock::time_point first_update_time_;
        size_t update_count_ = 0;
    };

public:
    ZedImuReceiver(std::shared_ptr<rclcpp::Node> node = nullptr,
                   const std::function<void(const IMUData&)>& callback = nullptr)
    {
        if (!node) {
            // 如果没有传入节点，创建独立的ROS上下文
            static int argc = 1;
            static char* argv[] = {(char*)"zed_imu_receiver"};
            if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
                rclcpp::init(argc, argv);
            }
            node = std::make_shared<rclcpp::Node>("zed_imu_receiver_node");
        }
        
        imu_node_ = std::make_shared<ZedImuNode>(callback);
        
        // 启动ROS线程
        spin_thread_ = std::thread([this]() {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(imu_node_);
            executor.spin();
        });
        
        std::cout << "ZED IMU接收器初始化完成" << std::endl;
    }
    
    ~ZedImuReceiver()
    {
        if (spin_thread_.joinable()) {
            rclcpp::shutdown();
            spin_thread_.join();
        }
    }
    
    // 获取最新IMU数据
    IMUData read() const
    {
        if (!imu_node_) {
            std::cerr << "错误: IMU接收器未初始化" << std::endl;
            return IMUData();
        }
        return imu_node_->getLatestData();
    }
    
    // 检查数据是否有效
    bool isDataValid() const
    {
        if (!imu_node_) return false;
        return imu_node_->isDataValid();
    }
    
    // 等待数据可用
    bool waitForData(int timeout_ms = 5000)
    {
        auto start = std::chrono::steady_clock::now();
        
        while (!imu_node_->isDataValid()) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            
            if (elapsed.count() > timeout_ms) {
                std::cerr << "等待IMU数据超时 (" << timeout_ms << "ms)" << std::endl;
                return false;
            }
            
            std::cout << "等待IMU数据..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "IMU数据已准备就绪" << std::endl;
        return true;
    }

    // 获取更新频率
    double getFrequency() const
    {
        if (!imu_node_) return 0.0;
        return imu_node_->getFrequency();
    }
    
private:
    std::shared_ptr<ZedImuNode> imu_node_;
    std::thread spin_thread_;
};

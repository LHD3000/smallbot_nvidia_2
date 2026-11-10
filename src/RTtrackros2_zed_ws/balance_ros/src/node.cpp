#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trackmsg/msg/balance_date.hpp>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include "balance_ros/utils.hpp"
#include "balance_ros/zed_imu_receiver.hpp"
#include "asio_driver/device/rc_sbus.hpp"
#include "asio_driver/io/timer.hpp"
#include "asio_driver/utils/range.hpp"
#include "asio_driver_special/motor_brt_can.hpp"
#include "asio_driver_special/motor_dc_io.hpp"
#include "ctrl_alg/filter/window_filter.hpp"
#include "ctrl_alg/pid/pid_pos.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>

static constexpr double rate = 200.0;
static constexpr double WHEEL_RADIUS = 0.127;

class BalanceController : public rclcpp::Node
{
private:
    rclcpp::Publisher<trackmsg::msg::BalanceDate>::SharedPtr date_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rc_sub_;
    double rc_last_time_ = 0.0;
    bool rc_connected_ = false;
    float last_target_speed_ = 0.0f;
    float last_target_yaw_ = 0.0f;
    std::shared_ptr<ZedImuReceiver> imu_receiver_;
    std::mutex mutex_;
    double target_speed_;
    double target_yaw_rate_;
    double last_cmd_time_;
    double battery_soc_ = 0.0;
    double battery_temp_ = 0.0;
    std::mutex battery_mutex_;
    LowPassFilter rc_throttle_filter{2.0, 0.02};
    LowPassFilter rc_yaw_filter{2.0, 0.02};
    float last_raw_throttle_ = 0.0f;
    float last_raw_yaw_ = 0.0f;
    bool rc_first_frame_ = true;
    bool dock_mode_ = false;

public:
    BalanceController()
        : rclcpp::Node("balance_controller"), target_speed_(0), target_yaw_rate_(0), last_cmd_time_(0)
    {
        date_pub_ = this->create_publisher<trackmsg::msg::BalanceDate>("/balance_date_b1",10);
        rc_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/rc_input_b1", 10,
            std::bind(&BalanceController::rcCallback, this, std::placeholders::_1));
        target_speed_=0;
        target_yaw_rate_=0;
    }

    void rcCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        rc_last_time_ = this->now().seconds();
        rc_connected_ = true;
        if (msg->axes.size() < 3) return;

        float raw_throttle = msg->axes[2];
        float raw_yaw = msg->axes[0];
        if (rc_first_frame_) {
            last_raw_throttle_ = raw_throttle;
            last_raw_yaw_ = raw_yaw;
            rc_first_frame_ = false;
        } else {
            if (std::abs(raw_throttle - last_raw_throttle_) > 0.3f)
                raw_throttle = last_raw_throttle_;
            if (std::abs(raw_yaw - last_raw_yaw_) > 0.3f)
                raw_yaw = last_raw_yaw_;
            last_raw_throttle_ = raw_throttle;
            last_raw_yaw_ = raw_yaw;
        }

        float throttle_pos = (raw_throttle + 1.0f) * 0.5f;
        throttle_pos = rc_throttle_filter.update(throttle_pos);
        raw_yaw = rc_yaw_filter.update(raw_yaw);

        const float throttle_deadzone = 0.05f;
        if (throttle_pos < throttle_deadzone)
            throttle_pos = 0.0f;
        else
            throttle_pos = (throttle_pos - throttle_deadzone) / (1.0f - throttle_deadzone);

        const float yaw_deadzone = 0.12f;
        float abs_yaw = std::abs(raw_yaw);
        if (abs_yaw < yaw_deadzone)
            raw_yaw = 0.0f;
        else
            raw_yaw = std::copysign((abs_yaw - yaw_deadzone) / (1.0f - yaw_deadzone), raw_yaw);

        float throttle = std::pow(throttle_pos, 1.0f);
        float yaw = std::copysign(std::pow(std::abs(raw_yaw), 1.0f), raw_yaw);

        int gear = 0;
        if (msg->axes.size() > 5) {
            float gear_val = msg->axes[5];
            if (gear_val > 0.5f)         gear = -1;
            else if (gear_val < -0.5f) gear = 1;
            else                        gear = 0;
        }

        float speed_cmd = throttle * 2.0f * gear;
        float yaw_cmd   = -yaw * 2.0f;

        float speed_step = 0.005f;
        float yaw_step   = 0.05f;
        speed_cmd = std::clamp(speed_cmd, last_target_speed_ - speed_step, last_target_speed_ + speed_step);
        yaw_cmd   = std::clamp(yaw_cmd, last_target_yaw_ - yaw_step, last_target_yaw_ + yaw_step);
        last_target_speed_ = speed_cmd;
        last_target_yaw_ = yaw_cmd;
        target_speed_ = speed_cmd;
        target_yaw_rate_ = (yaw_cmd * target_speed_) * 180.0 / M_PI;

        if (msg->buttons.size() > 3) {
            dock_mode_ = (msg->buttons[3] == 1);
        }
    }

    void run()
    {
        int angleTick = 0;
        int speedTick = 0;
        int heart_beat_tick = 0;
        float gyro = 0.0f;
        float angle = 0.0f;
        float gyroZ = 0.0f;
        float rpm_right = 0.0f;
        float rpm_left = 0.0f;
        float current_speed = 0.0f;
        float angleAdjust = 0.0f;
        float target_angle = 0.0f;
        float target_gyro = 0.0f;
        float forwardVal = 0.0f;
        float yawAdjust = 0.0f;
        float speed_angle_adjust = 0.0f;
        float speed_feedback = 0.0f;
        float left = 0.0f;
        float right = 0.0f;
        float kleft = 1.0f;
        float kright = 1.07f;
        float banlanceOffset = 0.0f;

        // ========== 对接过渡阶段状态 ==========
        int dock_transition_tick = 0;   // 切换到dock后的tick计数
        bool last_dock_mode = false;    // 上一帧dock状态，用于检测跳变
        const int TRANSITION_TICKS = 100; // 低头过渡时长：50 tick = 0.25s @200Hz
        const float HEAD_DOWN_CURRENT = -800.0f;  // 低头电流（负=反转=前倾）
        const float DOCK_THRUST = 2500.0f;         // 对接正电流

        // 数据记录文件
        std::ofstream angle_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/angle_data.txt");
        std::ofstream timestamp_angle_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_angle.txt");
        std::ofstream target_angle_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/target_angle_data.txt");
        std::ofstream timestamp_target_angle_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_target_angle.txt");
        std::ofstream speed_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/speed_data.txt");
        std::ofstream timestamp_speed_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_speed.txt");
        std::ofstream target_speed_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/target_speed_data.txt");
        std::ofstream timestamp_target_speed_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_target_speed.txt");
        std::ofstream gyro_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/gyro_data.txt");
        std::ofstream timestamp_gyro_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_gyro.txt");
        std::ofstream target_gyro_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/target_gyro_data.txt");
        std::ofstream timestamp_target_gyro_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_target_gyro.txt");
        std::ofstream speed_raw_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/speed_raw_data.txt");
        std::ofstream timestamp_speed_raw_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_speed_raw.txt");
        std::ofstream angle_filtered_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/angle_filtered_data.txt");
        std::ofstream timestamp_angle_filtered_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_angle_filtered.txt");
        std::ofstream gyro_filtered_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/gyro_filtered_data.txt");
        std::ofstream timestamp_gyro_filtered_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_gyro_filtered.txt");
        std::ofstream left_current_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/left_current_data.txt");
        std::ofstream timestamp_left_current_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_left_current.txt");
        std::ofstream right_current_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/right_current_data.txt");
        std::ofstream timestamp_right_current_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_right_current.txt");
        std::ofstream left_rpm_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/left_rpm_data.txt");
        std::ofstream right_rpm_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/right_rpm_data.txt");
        std::ofstream timestamp_rpm_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_rpm.txt");
        std::ofstream gyro_z_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/gyro_z_data.txt");
        std::ofstream timestamp_gyro_z_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_gyro_z.txt");
        std::ofstream gyro_z_filtered_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/gyro_z_filtered_data.txt");
        std::ofstream timestamp_gyro_z_filtered_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_gyro_z_filtered.txt");
        std::ofstream target_yaw_rate_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/target_yaw_rate_data.txt");
        std::ofstream timestamp_target_yaw_rate_file("/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_target_yaw_rate.txt");

        if(!angle_file) {
            std::cerr<<"cannot open "<<std::endl;
            return;
        }

        std::cout << "初始化设备..." << std::endl;
        AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
        std::unique_ptr<AD::Can> can = std::make_unique<AD::Can>(context);
        AD::CanBus::SharePtr can_bus = AD::CanBus::Create(context);
        AD::Special::MotorBrtCan motorLeft = AD::Special::MotorBrtCan(context);
        AD::Special::MotorBrtCan motorRight = AD::Special::MotorBrtCan(context);

        imu_receiver_ = std::make_shared<ZedImuReceiver>(
            shared_from_this(),
            [this](const ZedImuReceiver::IMUData&) {});

        motorLeft.Open(can_bus, AD::Special::MotorBrtCan::MotorConfig{.address = 5, .poleCount = 15});
        motorRight.Open(can_bus, AD::Special::MotorBrtCan::MotorConfig{.address = 6, .poleCount = 15});
        can->Open("can0");
        can_bus->Open(std::move(can));

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (!imu_receiver_->waitForData(1000)) {
            std::cerr << "ZED IMU 数据获取超时" << std::endl;
            return;
        }

        CtrlAlg::PIDPos<double> gyroPID(400 ,0.1, 25);
        CtrlAlg::PIDPos<double> anglePID(9, 0.01, 1);
        CtrlAlg::PIDPos<double> gyroZPID(160, 0.1, 1);
        CtrlAlg::PIDPos<double> speedPID(8.0, 0.002, 0.015);

        LowPassFilter speedFilter(50.0, 1.0 / rate);
        LowPassFilter gyroFilter(15.0, 1.0 / rate);
        LowPassFilter angleFilter(10.0, 1.0 / rate);
        LowPassFilter yawFilter(1.0, 1.0 / rate);
        LowPassFilter targetAngleFilter(50.0, 1.0 / rate);
        LowPassFilter speedOutputFilter(50.0, 1.0 / rate);
        SoftLimter WheelLimiter(3000, 8000);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(shared_from_this());
        rclcpp::Rate loop_rate(rate);
        int imu_failure_count = 0;
        const int max_imu_failures = 10;
        int data_count = 0;

        while (rclcpp::ok() && context->Running())
        {
            executor.spin_some();
            context->SpinOnce();

            double now=this->now().seconds();
            if(now - rc_last_time_ > 0.5) {
                rc_connected_=false;
                target_speed_=0;
                target_yaw_rate_=0;
                motorLeft.SetCurrent(0);
                motorRight.SetCurrent(0);
                loop_rate.sleep();
                continue;
            }

            if (!imu_receiver_->isDataValid()) {
                imu_failure_count++;
                if (imu_failure_count >= max_imu_failures) return;
                trackmsg::msg::BalanceDate date_msg;
                date_pub_->publish(date_msg);
                loop_rate.sleep();
                continue;
            }

            imu_failure_count = 0;
            auto imuDate = imu_receiver_->read();
            gyro = AD::Utils::InRange(imuDate.gyro[1], -100, 100) ? imuDate.gyro[1] : gyro;
            angle = AD::Utils::InRange(imuDate.angle[1], -30, 30) ? imuDate.angle[1] : angle;
            gyroZ = AD::Utils::InRange(imuDate.gyro[2], -60, 60) ? imuDate.gyro[2] : gyroZ;

            double current_angle_timestamp = this->now().seconds();
            angle_file << angle << std::endl;
            timestamp_angle_file << std::fixed << std::setprecision(9) << current_angle_timestamp << std::endl;
            data_count++;
            double current_gyro_timestamp = this->now().seconds();
            gyro_file << gyro << std::endl;
            timestamp_gyro_file << std::fixed << std::setprecision(9) << current_gyro_timestamp << std::endl;

            gyro = gyroFilter.update(gyro);
            angle = angleFilter.update(angle);
            gyroZ = yawFilter.update(gyroZ);

            double imu_filtered_ts = this->now().seconds();
            angle_filtered_file << angle << std::endl;
            timestamp_angle_filtered_file << std::fixed << std::setprecision(9) << imu_filtered_ts << std::endl;
            gyro_filtered_file << gyro << std::endl;
            timestamp_gyro_filtered_file << std::fixed << std::setprecision(9) << imu_filtered_ts << std::endl;

            // ========== 速度环 50Hz ==========
            speedTick++;
            if(speedTick >= 4) {
                speedTick = 0;
                motorLeft.RequestRPM();
                motorRight.RequestRPM();
                rpm_left = motorLeft.ReadState().rpm;
                rpm_right = motorRight.ReadState().rpm;
                speed_feedback = ((rpm_left + rpm_right) / 2.0 / 60.0) * 2 * M_PI * WHEEL_RADIUS;
                double rpm_ts = this->now().seconds();
                left_rpm_file << rpm_left << std::endl;
                right_rpm_file << rpm_right << std::endl;
                timestamp_rpm_file << std::fixed << std::setprecision(9) << rpm_ts << std::endl;
                current_speed = speedFilter.update(speed_feedback);

                if (!dock_mode_) {
                    double speed_error = target_speed_ - current_speed;
                    speed_angle_adjust = speedPID.GetControl(speed_error, 1000.0 / 50.0);
                }
            }

            float speed_angle_adjust_smooth = speedOutputFilter.update(speed_angle_adjust);
            double speed_raw_ts = this->now().seconds();
            speed_raw_file << current_speed << std::endl;
            timestamp_speed_raw_file << std::fixed << std::setprecision(9) << speed_raw_ts << std::endl;
            double current_speed_timestamp = this->now().seconds();
            speed_file << current_speed << std::endl;
            timestamp_speed_file << std::fixed << std::setprecision(9) << current_speed_timestamp << std::endl;

            // 安全措施
            if (std::abs(angle) > 20) {
                std::cerr << "角度超限: " << angle << std::endl;
                return;
            }
            if (std::abs(current_speed) > 3) {
                std::cerr << "速度超限: " << current_speed << std::endl;
                return;
            }
            if (std::abs(gyroZ) > 90) {
                std::cerr << "偏航角速度超限: " << gyroZ << std::endl;
                return;
            }

            // ================================================================
            // 模式分支
            // ================================================================
            if (dock_mode_)
            {
                // 检测自平衡→dock切换沿
                if (!last_dock_mode) {
                    dock_transition_tick = 0;
                    last_dock_mode = true;
                    std::cout << "[dock] 进入对接模式，先低头后顶进" << std::endl;
                }
                dock_transition_tick++;

                angleAdjust = 0.0f;
                speed_angle_adjust = 0.0f;
                speed_angle_adjust_smooth = 0.0f;
                target_gyro = 0.0f;
                forwardVal = 0.0f;
                yawAdjust = 0.0f;
                target_angle = banlanceOffset;

                // ====== 两段式输出：先低头（负电流），再顶进（正电流） ======
                if (dock_transition_tick < TRANSITION_TICKS) {
                    // 第一阶段：负电流，让车体前倾（低头）
                    left = HEAD_DOWN_CURRENT;
                    right = HEAD_DOWN_CURRENT;
                } else {
                    // 第二阶段：正电流，顶着前车往里走
                    left = DOCK_THRUST;
                    right = DOCK_THRUST;
                }
            }
            else
            {
                last_dock_mode = false;

                // ========== 正常自平衡PID计算 ==========
                double target_speed_timestamp = this->now().seconds();
                target_speed_file << target_speed_ << std::endl;
                timestamp_target_speed_file << std::fixed << std::setprecision(9) << target_speed_timestamp << std::endl;

                angleAdjust = speed_angle_adjust_smooth;
                angleAdjust = AD::Utils::ClampAbs(angleAdjust, 8.0);
                target_angle = banlanceOffset + angleAdjust;
                target_angle = targetAngleFilter.update(target_angle);

                double target_angle_timestamp = this->now().seconds();
                target_angle_file << target_angle << std::endl;
                timestamp_target_angle_file << std::fixed << std::setprecision(9) << target_angle_timestamp << std::endl;

                angleTick++;
                if (angleTick >= 2) {
                    angleTick = 0;
                    target_gyro = anglePID.GetControl(target_angle - angle, 1000.0 / 100.0);
                }

                forwardVal = -gyroPID.GetControl((target_gyro - gyro)* 1, 1000.0 / rate);
                forwardVal = AD::Utils::ClampAbs(forwardVal, 15000);

                yawAdjust = -gyroZPID.GetControl(target_yaw_rate_ - gyroZ, 1000.0 / rate);
                yawAdjust = AD::Utils::ClampAbs(yawAdjust, 4000);

                left = forwardVal + yawAdjust;
                right = forwardVal - yawAdjust;
            }

            double target_gyro_timestamp = this->now().seconds();
            target_gyro_file << target_gyro << std::endl;
            timestamp_target_gyro_file << std::fixed << std::setprecision(9) << target_gyro_timestamp << std::endl;

            double current_gyro_z_timestamp = this->now().seconds();
            gyro_z_file << gyroZ << std::endl;
            timestamp_gyro_z_file << std::fixed << std::setprecision(9)<< current_gyro_z_timestamp << std::endl;
            double gyro_z_filtered_ts = this->now().seconds();
            gyro_z_filtered_file << gyroZ << std::endl;
            timestamp_gyro_z_filtered_file << std::fixed << std::setprecision(9) << gyro_z_filtered_ts << std::endl;
            double target_yaw_rate_ts = this->now().seconds();
            target_yaw_rate_file << target_yaw_rate_ << std::endl;
            timestamp_target_yaw_rate_file << std::fixed << std::setprecision(9) << target_yaw_rate_ts << std::endl;

            // ========== 电机输出 ==========
            left = kleft * WheelLimiter.clamp(left);
            right = kright * WheelLimiter.clamp(right);
            motorLeft.SetCurrent(left);
            motorRight.SetCurrent(right);

            heart_beat_tick++;
            if(heart_beat_tick >= 10) {
                heart_beat_tick = 0;
                motorLeft.SendHeartBeat();
                motorRight.SendHeartBeat();
            }

            double current_ts = this->now().seconds();
            left_current_file << left << std::endl;
            timestamp_left_current_file << std::fixed << std::setprecision(9) << current_ts << std::endl;
            right_current_file << right << std::endl;
            timestamp_right_current_file << std::fixed << std::setprecision(9) << current_ts << std::endl;

            trackmsg::msg::BalanceDate date_msg;
            date_msg.gyro = gyro;
            date_msg.angle = angle;
            date_msg.gyro_z = gyroZ;
            date_msg.current_speed = current_speed;
            date_msg.target_angle = target_angle;
            date_msg.target_gyro = target_gyro;
            date_msg.left = left;
            date_msg.right = right;
            date_msg.target_speed = target_speed_;
            date_msg.target_yaw_rate = target_yaw_rate_;
            date_msg.battery_soc = 0;
            date_msg.battery_temperature = 0;
            date_pub_->publish(date_msg);

            loop_rate.sleep();
        }

        angle_file.close();
        timestamp_angle_file.close();
        target_angle_file.close();
        timestamp_target_angle_file.close();
        speed_file.close();
        timestamp_speed_file.close();
        target_speed_file.close();
        timestamp_target_speed_file.close();
        gyro_file.close();
        timestamp_gyro_file.close();
        target_gyro_file.close();
        timestamp_target_gyro_file.close();
        left_current_file.close();
        timestamp_left_current_file.close();
        right_current_file.close();
        timestamp_right_current_file.close();
        speed_raw_file.close();
        timestamp_speed_raw_file.close();
        angle_filtered_file.close();
        timestamp_angle_filtered_file.close();
        gyro_filtered_file.close();
        timestamp_gyro_filtered_file.close();
        left_rpm_file.close();
        right_rpm_file.close();
        timestamp_rpm_file.close();
        gyro_z_file.close();
        timestamp_gyro_z_file.close();
        gyro_z_filtered_file.close();
        timestamp_gyro_z_filtered_file.close();
        target_yaw_rate_file.close();
        timestamp_target_yaw_rate_file.close();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<BalanceController>();
    controller->run();
    rclcpp::shutdown();
    return 0;
}

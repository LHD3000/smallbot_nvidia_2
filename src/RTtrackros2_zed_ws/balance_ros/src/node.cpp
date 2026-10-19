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
// static constexpr uint32_t BATTERY_QUERY_ID = 0x0400FF80;
// static constexpr uint32_t BATTERY_SOC_ID   = 0x04028001;
// static constexpr uint32_t BATTERY_TEMP_ID  = 0x04058001;


class BalanceController : public rclcpp::Node
{
private:
    rclcpp::Publisher<trackmsg::msg::BalanceDate>::SharedPtr date_pub_;

    //========== RC ==========
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
    bool battery_soc_valid_ = false;
    bool battery_temp_valid_ = false;
    double battery_soc_last_update_ = 0.0;
    double battery_temp_last_update_ = 0.0;
    std::mutex battery_mutex_;

    // RC输入滤波：2Hz截止+单帧跳变抑制
    LowPassFilter rc_throttle_filter{2.0, 0.02};
    LowPassFilter rc_yaw_filter{2.0, 0.02};
    float last_raw_throttle_ = 0.0f;
    float last_raw_yaw_ = 0.0f;
    bool rc_first_frame_ = true;

public:
    BalanceController()
        : rclcpp::Node("balance_controller"), target_speed_(0), target_yaw_rate_(0), last_cmd_time_(0)
    {
        date_pub_ =
        this->create_publisher<trackmsg::msg::BalanceDate>(
        "/balance_date",10);
        rc_sub_ =
        this->create_subscription<sensor_msgs::msg::Joy>(
        "/rc_input",
        10,
        std::bind(
        &BalanceController::rcCallback,
        this,
        std::placeholders::_1));
        target_speed_=0;
        target_yaw_rate_=0;
    }

    void rcCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        rc_last_time_ = this->now().seconds();
        rc_connected_ = true;

      
        if (msg->axes.size() < 3)
            return;

        // ========== 1. 单帧跳变抑制（作用于原始轴值） ==========
              // ========== 1. 单帧跳变抑制（作用于原始轴值） ==========
        float raw_throttle = msg->axes[2];
        float raw_yaw = msg->axes[0];
        if (rc_first_frame_) {
            // 首帧直接用真实值初始化，避免最低点静止时被永久抑制
            last_raw_throttle_ = raw_throttle;
            last_raw_yaw_ = raw_yaw;
            rc_first_frame_ = false;
        } else {
            if (std::abs(raw_throttle - last_raw_throttle_) > 0.3f) {
                raw_throttle = last_raw_throttle_;
            }
            if (std::abs(raw_yaw - last_raw_yaw_) > 0.3f) {
                raw_yaw = last_raw_yaw_;
            }
            last_raw_throttle_ = raw_throttle;
            last_raw_yaw_ = raw_yaw;
        }


        // ========== 2. 油门映射：最低点(-1)→0，最高点(1)→1 ==========
        float throttle_pos = (raw_throttle + 1.0f) * 0.5f;
        // 油门低通滤波（作用于正向量）
        throttle_pos = rc_throttle_filter.update(throttle_pos);
        // 偏航低通滤波（保持原逻辑）
        raw_yaw = rc_yaw_filter.update(raw_yaw);

        // ========== 3. 死区 ==========
        // 油门死区：最低点附近 5% 视为静止
        const float throttle_deadzone = 0.05f;
        if (throttle_pos < throttle_deadzone) {
            throttle_pos = 0.0f;
        } else {
            throttle_pos = (throttle_pos - throttle_deadzone) / (1.0f - throttle_deadzone);
        }
        // 偏航死区（保持原逻辑）
        const float yaw_deadzone = 0.12f;
        float abs_yaw = std::abs(raw_yaw);
        if (abs_yaw < yaw_deadzone) {
            raw_yaw = 0.0f;
        } else {
            raw_yaw = std::copysign((abs_yaw - yaw_deadzone) / (1.0f - yaw_deadzone), raw_yaw);
        }

        // ========== 4. 曲线（线性，指数=1.0） ==========
        float throttle = std::pow(throttle_pos, 1.0f);
        float yaw = std::copysign(std::pow(std::abs(raw_yaw), 1.0f), raw_yaw);

        // ========== 5. 三档开关：axes[5]，1=前进，0=空挡，-1=倒车 ==========
        int gear = 0;
        if (msg->axes.size() > 5) {
            float gear_val = msg->axes[5];
            if (gear_val > 0.5f)       gear = -1;    // 上档：前进
            else if (gear_val < -0.5f) gear = 1;   // 下档：倒车
            else                        gear = 0;    // 中档：空挡
        }

        // ========== 6. 速度映射 + 档位方向 ==========
        float speed_cmd = throttle * 2.0f * gear;   // gear=0 时空挡静止
        float yaw_cmd   = -yaw * 2.0f;

        // ========== 7. 斜率限制（保持原逻辑） ==========
        float speed_step = 0.005f;
        float yaw_step   = 0.05f;
        speed_cmd = std::clamp(speed_cmd,
                         last_target_speed_ - speed_step,
                         last_target_speed_ + speed_step);
        yaw_cmd = std::clamp(yaw_cmd,
                         last_target_yaw_ - yaw_step,
                         last_target_yaw_ + yaw_step);
        last_target_speed_ = speed_cmd;
        last_target_yaw_ = yaw_cmd;
        target_speed_ = speed_cmd;
        target_yaw_rate_ = (yaw_cmd * target_speed_) * 180.0 / M_PI;
    }

    void run()
    {
        int angleTick = 0;
        int speedTick = 0;
        int heart_beat_tick = 0;  // CAN心跳分频计数器
        float gyro = 0.0;
        float angle = 0.0;
        float gyroZ = 0.0;
        float rpm_right = 0.0;
        float rpm_left = 0.0;
        float current_speed = 0.0;
        float angleAdjust = 0.0;
        float target_angle = 0.0;
        float target_gyro = 0.0;
        float forwardVal = 0.0;
        float yawAdjust = 0.0;
        float speed_angle_adjust = 0.0;
        float speed_feedback = 0.0;
        float left = 0.0;
        float right = 0.0;
        float kleft = 1.0;
        float kright = 1.07;
        float banlanceOffset = -4.0;

        auto last_battery_query =
        std::chrono::steady_clock::now();

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

        if(!angle_file)
        {
            std::cerr<<"cannot open "<<std::endl;
            return;
        }

        std::cout << "初始化设备..." << std::endl;
        AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
        std::unique_ptr<AD::Can> can = std::make_unique<AD::Can>(context);
        AD::CanBus::SharePtr can_bus = AD::CanBus::Create(context);
        AD::Special::MotorBrtCan motorLeft = AD::Special::MotorBrtCan(context);
        AD::Special::MotorBrtCan motorRight = AD::Special::MotorBrtCan(context);

        // 初始化 ZED IMU
        imu_receiver_ = std::make_shared<ZedImuReceiver>(
            shared_from_this(),
            [this](const ZedImuReceiver::IMUData&) {
            }
        );

        motorLeft.Open(can_bus, AD::Special::MotorBrtCan::MotorConfig{.address = 5, .poleCount = 15});
        motorRight.Open(can_bus, AD::Special::MotorBrtCan::MotorConfig{.address = 6, .poleCount = 15});
        can->Open("can0");
        can_bus->Open(std::move(can));

        // 电池SOC回调
        // can_bus->AddListener(
        // BATTERY_SOC_ID,
        // [this](std::span<const unsigned char> data)
        // {
        //     if (data.size() < 6) return;
        //     const uint16_t soc_raw =
        //         (static_cast<uint16_t>(data[4]) << 8) |
        //         static_cast<uint16_t>(data[5]);
        //     double soc = static_cast<double>(soc_raw) * 0.1;
        //     soc = std::clamp(soc, 0.0, 100.0);
        //     {
        //         std::lock_guard<std::mutex> lock(battery_mutex_);
        //         battery_soc_ = soc;
        //         battery_soc_valid_ = true;
        //     }
        // });
        // 电池温度回调
        // can_bus->AddListener(
        // BATTERY_TEMP_ID,
        // [this](std::span<const unsigned char> data)
        // {
        //     if (data.size() < 1) return;
        //     const double temperature = static_cast<double>(data[0]) - 40.0;
        //     {
        //         std::lock_guard<std::mutex> lock(battery_mutex_);
        //         battery_temp_ = temperature;
        //         battery_temp_valid_ = true;
        //     }
        // });

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (!imu_receiver_->waitForData(500)) {
            std::cerr << "ZED IMU 数据获取超时" << std::endl;
            return;
        }

        // ========== PID控制器 ==========
        CtrlAlg::PIDPos<double> gyroPID(380 ,0.1, 25);
        CtrlAlg::PIDPos<double> anglePID(9, 0.01, 1);
       // CtrlAlg::PIDPos<double> gyroZPID(0, 0, 0);
        CtrlAlg::PIDPos<double> gyroZPID(160, 0.1, 1);
        // 速度环参数匹配50Hz控制周期
        CtrlAlg::PIDPos<double> speedPID(8.0, 0.002, 0.015);

        // ========== 滤波器 ==========
        LowPassFilter speedFilter(50.0, 1.0 / rate);
        LowPassFilter gyroFilter(15.0, 1.0 / rate);
        LowPassFilter angleFilter(10.0, 1.0 / rate);
        LowPassFilter yawFilter(1.0, 1.0 / rate);
        LowPassFilter targetAngleFilter(50.0, 1.0 / rate);
        LowPassFilter targetyawRateFilter(5.0,1.0/rate);
        LowPassFilter speedOutputFilter(50.0, 1.0 / rate);
        SoftLimter WheelLimiter(5000, 15000);

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

            // 电池查询
            // const auto battery_query_now = std::chrono::steady_clock::now();
            // if (std::chrono::duration_cast<std::chrono::seconds>(battery_query_now - last_battery_query).count() >= 2)
            // {
            //     static const std::array<unsigned char, 1> query_data = {0x00};
            //     can_bus->Send(BATTERY_QUERY_ID, query_data);
            //     last_battery_query = battery_query_now;
            // }


            double now=this->now().seconds();
            if(now - rc_last_time_ > 0.3)
            {
                rc_connected_=false;
                target_speed_=0;
                target_yaw_rate_=0;
                motorLeft.SetCurrent(0);
                motorRight.SetCurrent(0);
                loop_rate.sleep();
                continue;
            }

            // IMU有效性检查
            if (!imu_receiver_->isDataValid()) {
                imu_failure_count++;
                if (imu_failure_count >= max_imu_failures) return;

                trackmsg::msg::BalanceDate date_msg;
                date_msg.gyro = 0;
                date_msg.angle = 0;
                date_msg.gyro_z = 0;
                date_msg.current_speed = 0;
                date_msg.target_angle = 0;
                date_msg.target_gyro = 0;
                date_msg.left = 0;
                date_msg.right = 0;
                date_msg.target_speed = 0;
                date_msg.target_yaw_rate = 0;
                date_pub_->publish(date_msg);

                loop_rate.sleep();
                continue;
            }

            imu_failure_count = 0;
            auto imuDate = imu_receiver_->read();

            gyro = AD::Utils::InRange(imuDate.gyro[1], -100, 100) ? imuDate.gyro[1] : gyro;
            angle = AD::Utils::InRange(imuDate.angle[1], -30, 30) ? imuDate.angle[1] : angle;
            gyroZ = AD::Utils::InRange(imuDate.gyro[2], -60, 60) ? imuDate.gyro[2] : gyroZ;

            // 记录原始数据
            double current_angle_timestamp = this->now().seconds();
            angle_file << angle << std::endl;
            timestamp_angle_file << std::fixed << std::setprecision(9) << current_angle_timestamp << std::endl;
            data_count++;

            double current_gyro_timestamp = this->now().seconds();
            gyro_file << gyro << std::endl;
            timestamp_gyro_file << std::fixed << std::setprecision(9) << current_gyro_timestamp << std::endl;

            // IMU滤波
            gyro = gyroFilter.update(gyro);
            angle = angleFilter.update(angle);
            gyroZ = yawFilter.update(gyroZ);

            double imu_filtered_ts = this->now().seconds();
            angle_filtered_file << angle << std::endl;
            timestamp_angle_filtered_file << std::fixed << std::setprecision(9) << imu_filtered_ts << std::endl;
            gyro_filtered_file << gyro << std::endl;
            timestamp_gyro_filtered_file << std::fixed << std::setprecision(9) << imu_filtered_ts << std::endl;

            // ========== 速度环 50Hz：每4个主控周期执行一次 ==========
            speedTick++;
            if(speedTick >= 4)
            {
                speedTick = 0;
                motorLeft.RequestRPM();
                motorRight.RequestRPM();
                rpm_left = motorLeft.ReadState().rpm;
                rpm_right = motorRight.ReadState().rpm;
                speed_feedback = ((rpm_left + rpm_right) / 2.0 / 60.0) * 2 * M_PI * WHEEL_RADIUS;

                double rpm_ts = this->now().seconds();
                left_rpm_file << rpm_left << std::endl;
                right_rpm_file << rpm_right << std::endl;
                timestamp_rpm_file << std::fixed << std::setprecision(9) << rpm_ts  << std::endl;

                current_speed = speedFilter.update(speed_feedback);

                // PID计算，控制周期20ms (50Hz)
                double speed_error = target_speed_ - current_speed;
                speed_angle_adjust = speedPID.GetControl(speed_error, 1000.0 / 50.0);

                // 输出限幅3度
                //speed_angle_adjust = AD::Utils::ClampAbs(speed_angle_adjust, 5.5);

                std::cout<<"目标速度："<<target_speed_<<std::endl;
            }

            // 速度环输出平滑滤波
            float speed_angle_adjust_smooth = speedOutputFilter.update(speed_angle_adjust);

            double speed_raw_ts = this->now().seconds();
            speed_raw_file << current_speed << std::endl;
            timestamp_speed_raw_file << std::fixed << std::setprecision(9) << speed_raw_ts << std::endl;

            double current_speed_timestamp = this->now().seconds();
            speed_file << current_speed << std::endl;
            timestamp_speed_file << std::fixed << std::setprecision(9) << current_speed_timestamp << std::endl;

            // 安全措施
            if (std::abs(angle) > 20)
            {
                std::cerr << "角度超限: " << angle << std::endl;
                return;
            }
            if (std::abs(current_speed) > 3)
            {
                std::cerr << "速度超限: " << current_speed << std::endl;
                return;
            }
            if (std::abs(gyroZ) > 90)
            {
                std::cerr << "偏航角速度超限: " << gyroZ << std::endl;
                return;
            }

            // 速度环输出作为角度环调整量
            double target_speed_timestamp = this->now().seconds();
            target_speed_file << target_speed_ << std::endl;
            timestamp_target_speed_file << std::fixed << std::setprecision(9) << target_speed_timestamp << std::endl;

            angleAdjust = speed_angle_adjust_smooth;
            angleAdjust = AD::Utils::ClampAbs(angleAdjust, 18.0);

            // 角度环
            target_angle = banlanceOffset + angleAdjust;
            target_angle = targetAngleFilter.update(target_angle);

            double target_angle_timestamp = this->now().seconds();
            target_angle_file << target_angle << std::endl;
            timestamp_target_angle_file << std::fixed << std::setprecision(9) << target_angle_timestamp << std::endl;

            angleTick++;
            if (angleTick >= 2)
            {
                angleTick = 0;
                target_gyro = anglePID.GetControl(target_angle - angle, 1000.0 / 100.0);
            }

            double target_gyro_timestamp = this->now().seconds();
            target_gyro_file << target_gyro << std::endl;
            timestamp_target_gyro_file << std::fixed << std::setprecision(9) << target_gyro_timestamp << std::endl;

            // 角速度环（200Hz，核心控制）
            forwardVal = -gyroPID.GetControl((target_gyro - gyro)* 1, 1000.0 / rate);
            forwardVal = AD::Utils::ClampAbs(forwardVal, 15000);

            // 转向环（200Hz）
            yawAdjust = -gyroZPID.GetControl(target_yaw_rate_ - gyroZ, 1000.0 / rate);
            yawAdjust = AD::Utils::ClampAbs(yawAdjust, 4000);

            // 记录偏航数据
            double current_gyro_z_timestamp = this->now().seconds();
            gyro_z_file << gyroZ << std::endl;
            timestamp_gyro_z_file << std::fixed << std::setprecision(9)<< current_gyro_z_timestamp << std::endl;

            double gyro_z_filtered_ts = this->now().seconds();
            gyro_z_filtered_file << gyroZ << std::endl;
            timestamp_gyro_z_filtered_file << std::fixed << std::setprecision(9) << gyro_z_filtered_ts << std::endl;

            double target_yaw_rate_ts = this->now().seconds();
            target_yaw_rate_file << target_yaw_rate_ << std::endl;
            timestamp_target_yaw_rate_file << std::fixed << std::setprecision(9) << target_yaw_rate_ts << std::endl;

            // 电机输出（电流指令保持200Hz，姿态控制核心）
            left = forwardVal + yawAdjust;
            right = forwardVal - yawAdjust;

            left = kleft * WheelLimiter.clamp(left);
            right = kright * WheelLimiter.clamp(right);
            motorLeft.SetCurrent(left);
            motorRight.SetCurrent(right);

            // ========== 电机心跳：20Hz，每10个主循环发一次 ==========
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

            // 发布数据
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

            double battery_soc = 0.0;
            double battery_temp = 0.0;
            {
                std::lock_guard<std::mutex> lock(battery_mutex_);
                battery_soc = battery_soc_;
                battery_temp = battery_temp_;
            }
            date_msg.battery_soc = static_cast<float>(battery_soc);
            date_msg.battery_temperature = static_cast<float>(battery_temp);
            date_pub_->publish(date_msg);

            loop_rate.sleep();
        }

        // 关闭文件
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

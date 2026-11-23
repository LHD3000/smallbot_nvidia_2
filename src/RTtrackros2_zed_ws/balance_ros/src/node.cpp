
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
#include <algorithm>

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

static constexpr double rate = 200.0;
static constexpr double WHEEL_RADIUS = 0.127;

// 对接模式每个电机最大输出电流
static constexpr float DOCK_MAX_CURRENT = 10000.0f;

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

    // 模式与遥控器状态
    bool dock_mode_ = false;
    int gear_direction_ = 0;
    float dock_throttle_ = 0.0f;

    // RC失联后必须先松开对接按钮，再允许重新进入对接模式
    bool dock_rearm_required_ = false;

public:
    BalanceController()
        : rclcpp::Node("balance_controller"),
          target_speed_(0),
          target_yaw_rate_(0),
          last_cmd_time_(0)
    {
        date_pub_ = this->create_publisher<trackmsg::msg::BalanceDate>(
            "/balance_date_b1", 10);

        rc_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/rc_input_b1",
            10,
            std::bind(
                &BalanceController::rcCallback,
                this,
                std::placeholders::_1));

        target_speed_ = 0;
        target_yaw_rate_ = 0;
    }

    void rcCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        rc_last_time_ = this->now().seconds();
        rc_connected_ = true;

        if (msg->axes.size() < 3)
            return;

        float raw_throttle = msg->axes[2];
        float raw_yaw = msg->axes[0];

        if (rc_first_frame_) {
            last_raw_throttle_ = raw_throttle;
            last_raw_yaw_ = raw_yaw;
            rc_first_frame_ = false;
        }
        else {
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
            throttle_pos =
                (throttle_pos - throttle_deadzone) /
                (1.0f - throttle_deadzone);

        const float yaw_deadzone = 0.12f;
        float abs_yaw = std::abs(raw_yaw);

        if (abs_yaw < yaw_deadzone)
            raw_yaw = 0.0f;
        else
            raw_yaw = std::copysign(
                (abs_yaw - yaw_deadzone) / (1.0f - yaw_deadzone),
                raw_yaw);

        float throttle = std::pow(throttle_pos, 1.0f);
        float yaw = std::copysign(
            std::pow(std::abs(raw_yaw), 1.0f),
            raw_yaw);

        int gear = 0;

        if (msg->axes.size() > 5) {
            float gear_val = msg->axes[5];

            if (gear_val > 0.5f)
                gear = -1;
            else if (gear_val < -0.5f)
                gear = 1;
            else
                gear = 0;
        }

        gear_direction_ = gear;
        dock_throttle_ = throttle_pos;

        float speed_cmd = throttle * 2.0f * gear;
        float yaw_cmd = -yaw * 2.0f;

        float speed_step = 0.005f;
        float yaw_step = 0.05f;

        speed_cmd = std::clamp(
            speed_cmd,
            last_target_speed_ - speed_step,
            last_target_speed_ + speed_step);

        yaw_cmd = std::clamp(
            yaw_cmd,
            last_target_yaw_ - yaw_step,
            last_target_yaw_ + yaw_step);

        last_target_speed_ = speed_cmd;
        last_target_yaw_ = yaw_cmd;

        target_speed_ = speed_cmd;

        target_yaw_rate_ =
            (yaw_cmd * target_speed_) * 180.0 / M_PI;

        // 原有按钮映射不变：buttons[3] 控制对接模式
        bool dock_button_pressed = false;

        if (msg->buttons.size() > 3)
            dock_button_pressed = (msg->buttons[3] == 1);

        // 失联后必须先松开按钮，避免恢复通信时自动恢复旧对接动作
        if (dock_rearm_required_) {
            if (!dock_button_pressed)
                dock_rearm_required_ = false;

            dock_mode_ = false;
        }
        else {
            dock_mode_ = dock_button_pressed;
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
        float speed_angle_adjust_smooth = 0.0f;
        float speed_feedback = 0.0f;

        float left = 0.0f;
        float right = 0.0f;

        float kleft = 1.0f;
        float kright = 1.07f;

        float banlanceOffset = 0.0f;

        // 对接模式状态
        bool last_dock_mode = false;
        bool dock_takeover = false;

        // 切换瞬间记录的左右轮实际输出电流绝对值之和
        float dock_initial_total_current = 0.0f;

        // 上一控制周期实际发送给电机的电流
        float last_applied_left = 0.0f;
        float last_applied_right = 0.0f;

        // 数据记录文件
        std::ofstream angle_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/angle_data.txt");

        std::ofstream timestamp_angle_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_angle.txt");

        std::ofstream target_angle_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/target_angle_data.txt");

        std::ofstream timestamp_target_angle_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_target_angle.txt");

        std::ofstream speed_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/speed_data.txt");

        std::ofstream timestamp_speed_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_speed.txt");

        std::ofstream target_speed_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/target_speed_data.txt");

        std::ofstream timestamp_target_speed_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_target_speed.txt");

        std::ofstream gyro_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/gyro_data.txt");

        std::ofstream timestamp_gyro_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_gyro.txt");

        std::ofstream target_gyro_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/target_gyro_data.txt");

        std::ofstream timestamp_target_gyro_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_target_gyro.txt");

        std::ofstream speed_raw_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/speed_raw_data.txt");

        std::ofstream timestamp_speed_raw_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_speed_raw.txt");

        std::ofstream angle_filtered_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/angle_filtered_data.txt");

        std::ofstream timestamp_angle_filtered_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_angle_filtered.txt");

        std::ofstream gyro_filtered_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/gyro_filtered_data.txt");

        std::ofstream timestamp_gyro_filtered_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_gyro_filtered.txt");

        std::ofstream left_current_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/left_current_data.txt");

        std::ofstream timestamp_left_current_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_left_current.txt");

        std::ofstream right_current_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/right_current_data.txt");

        std::ofstream timestamp_right_current_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_right_current.txt");

        std::ofstream left_rpm_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/left_rpm_data.txt");

        std::ofstream right_rpm_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/right_rpm_data.txt");

        std::ofstream timestamp_rpm_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_rpm.txt");

        std::ofstream gyro_z_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/gyro_z_data.txt");

        std::ofstream timestamp_gyro_z_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_gyro_z.txt");

        std::ofstream gyro_z_filtered_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/gyro_z_filtered_data.txt");

        std::ofstream timestamp_gyro_z_filtered_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_gyro_z_filtered.txt");

        std::ofstream target_yaw_rate_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/target_yaw_rate_data.txt");

        std::ofstream timestamp_target_yaw_rate_file(
            "/home/nvidia/jeston/smallbot_ws/src/RTtrackros2_zed_ws/balance_ros/src/timestamp_target_yaw_rate.txt");

        if (!angle_file) {
            std::cerr << "cannot open data files" << std::endl;
            return;
        }

        std::cout << "初始化设备..." << std::endl;

        AD::AsioContext::SharePtr context =
            AD::AsioContext::CreatAsioContext();

        std::unique_ptr<AD::Can> can =
            std::make_unique<AD::Can>(context);

        AD::CanBus::SharePtr can_bus =
            AD::CanBus::Create(context);

        AD::Special::MotorBrtCan motorLeft(context);
        AD::Special::MotorBrtCan motorRight(context);

        imu_receiver_ = std::make_shared<ZedImuReceiver>(
            shared_from_this(),
            [this](const ZedImuReceiver::IMUData&) {});

        motorLeft.Open(
            can_bus,
            AD::Special::MotorBrtCan::MotorConfig{
                .address = 5,
                .poleCount = 15});

        motorRight.Open(
            can_bus,
            AD::Special::MotorBrtCan::MotorConfig{
                .address = 6,
                .poleCount = 15});

        can->Open("can0");
        can_bus->Open(std::move(can));

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1000));

        if (!imu_receiver_->waitForData(1000)) {
            std::cerr << "ZED IMU 数据获取超时" << std::endl;
            return;
        }

        // ============================================================
        // PID控制器
        // 纵向串级PID采用unique_ptr，便于模式切换时重新初始化
        // ============================================================

        std::unique_ptr<CtrlAlg::PIDPos<double>> gyroPID;
        std::unique_ptr<CtrlAlg::PIDPos<double>> anglePID;
        std::unique_ptr<CtrlAlg::PIDPos<double>> speedPID;

        // 横摆PID独立运行，不随纵向串级PID停止
        CtrlAlg::PIDPos<double> gyroZPID(160, 0.1, 1);

        auto resetBalancePIDs = [&]() {
            gyroPID = std::make_unique<CtrlAlg::PIDPos<double>>(
                410, 0.1, 25);

            anglePID = std::make_unique<CtrlAlg::PIDPos<double>>(
                10, 0.01, 1);

            speedPID = std::make_unique<CtrlAlg::PIDPos<double>>(
                8.0, 0.002, 0.015);
        };

        resetBalancePIDs();

        // ============================================================
        // 滤波器
        // ============================================================

        LowPassFilter speedFilter(50.0, 1.0 / rate);
        LowPassFilter gyroFilter(15.0, 1.0 / rate);
        LowPassFilter angleFilter(10.0, 1.0 / rate);
        LowPassFilter yawFilter(1.0, 1.0 / rate);

        std::unique_ptr<LowPassFilter> targetAngleFilter =
            std::make_unique<LowPassFilter>(
                50.0, 1.0 / rate);

        std::unique_ptr<LowPassFilter> speedOutputFilter =
            std::make_unique<LowPassFilter>(
                50.0, 1.0 / rate);

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

            double now = this->now().seconds();

            // ========================================================
            // RC失联保护
            // ========================================================

            if (now - rc_last_time_ > 0.5) {

                if (rc_connected_) {
                    std::cerr << "RC LOST" << std::endl;
                }

                rc_connected_ = false;

                target_speed_ = 0;
                target_yaw_rate_ = 0;

                // 失联后退出对接状态
                dock_mode_ = false;
                dock_rearm_required_ = true;

                last_dock_mode = false;
                dock_takeover = false;
                dock_initial_total_current = 0.0f;

                last_applied_left = 0.0f;
                last_applied_right = 0.0f;

                motorLeft.SetCurrent(0);
                motorRight.SetCurrent(0);

                loop_rate.sleep();
                continue;
            }

            // ========================================================
            // IMU有效性检查
            // ========================================================

            if (!imu_receiver_->isDataValid()) {

                imu_failure_count++;

                if (imu_failure_count >= max_imu_failures) {
                    motorLeft.SetCurrent(0);
                    motorRight.SetCurrent(0);
                    return;
                }

                trackmsg::msg::BalanceDate date_msg;
                date_pub_->publish(date_msg);

                loop_rate.sleep();
                continue;
            }

            imu_failure_count = 0;

            auto imuDate = imu_receiver_->read();

            gyro = AD::Utils::InRange(
                imuDate.gyro[1], -100, 100)
                ? imuDate.gyro[1] : gyro;

            angle = AD::Utils::InRange(
                imuDate.angle[1], -30, 30)
                ? imuDate.angle[1] : angle;

            gyroZ = AD::Utils::InRange(
                imuDate.gyro[2], -60, 60)
                ? imuDate.gyro[2] : gyroZ;

            double current_angle_timestamp =
                this->now().seconds();

            angle_file << angle << std::endl;

            timestamp_angle_file
                << std::fixed << std::setprecision(9)
                << current_angle_timestamp << std::endl;

            data_count++;

            double current_gyro_timestamp =
                this->now().seconds();

            gyro_file << gyro << std::endl;

            timestamp_gyro_file
                << std::fixed << std::setprecision(9)
                << current_gyro_timestamp << std::endl;

            gyro = gyroFilter.update(gyro);
            angle = angleFilter.update(angle);
            gyroZ = yawFilter.update(gyroZ);

            double imu_filtered_ts =
                this->now().seconds();

            angle_filtered_file << angle << std::endl;

            timestamp_angle_filtered_file
                << std::fixed << std::setprecision(9)
                << imu_filtered_ts << std::endl;

            gyro_filtered_file << gyro << std::endl;

            timestamp_gyro_filtered_file
                << std::fixed << std::setprecision(9)
                << imu_filtered_ts << std::endl;

            // ========================================================
            // 速度反馈采集
            // 注意：对接模式下只读取速度，不计算纵向速度PID
            // ========================================================

            speedTick++;

            if (speedTick >= 4) {

                speedTick = 0;

                motorLeft.RequestRPM();
                motorRight.RequestRPM();

                rpm_left = motorLeft.ReadState().rpm;
                rpm_right = motorRight.ReadState().rpm;

                speed_feedback =
                    ((rpm_left + rpm_right) / 2.0 / 60.0)
                    * 2 * M_PI * WHEEL_RADIUS;

                double rpm_ts = this->now().seconds();

                left_rpm_file << rpm_left << std::endl;
                right_rpm_file << rpm_right << std::endl;

                timestamp_rpm_file
                    << std::fixed << std::setprecision(9)
                    << rpm_ts << std::endl;

                current_speed = speedFilter.update(speed_feedback);

                // 仅在自平衡模式下计算速度PID
                if (!dock_mode_) {

                    double speed_error =
                        target_speed_ - current_speed;

                    speed_angle_adjust =
                        speedPID->GetControl(
                            speed_error,
                            1000.0 / 50.0);
                }
            }

            double speed_raw_ts = this->now().seconds();

            speed_raw_file << current_speed << std::endl;

            timestamp_speed_raw_file
                << std::fixed << std::setprecision(9)
                << speed_raw_ts << std::endl;

            double current_speed_timestamp =
                this->now().seconds();

            speed_file << current_speed << std::endl;

            timestamp_speed_file
                << std::fixed << std::setprecision(9)
                << current_speed_timestamp << std::endl;

            // ========================================================
            // 安全保护
            // ========================================================

            if (std::abs(angle) > 20) {

                std::cerr
                    << "角度超限: " << angle
                    << std::endl;

                motorLeft.SetCurrent(0);
                motorRight.SetCurrent(0);
                return;
            }

            if (std::abs(current_speed) > 3) {

                std::cerr
                    << "速度超限: " << current_speed
                    << std::endl;

                motorLeft.SetCurrent(0);
                motorRight.SetCurrent(0);
                return;
            }

            if (std::abs(gyroZ) > 90) {

                std::cerr
                    << "偏航角速度超限: " << gyroZ
                    << std::endl;

                motorLeft.SetCurrent(0);
                motorRight.SetCurrent(0);
                return;
            }

            // ========================================================
            // 模式切换检测
            // ========================================================

            if (dock_mode_ && !last_dock_mode) {

                // 自平衡 -> 对接
                // 记录切换前最后一次实际发送的左右轮电流
                dock_initial_total_current =
                    std::abs(last_applied_left) +
                    std::abs(last_applied_right);

                dock_takeover = false;

                // 停止纵向串级PID的输出使用
                speed_angle_adjust = 0.0f;
                speed_angle_adjust_smooth = 0.0f;
                angleAdjust = 0.0f;
                target_gyro = 0.0f;
                forwardVal = 0.0f;

                std::cout
                    << "[dock] 进入对接模式"
                    << "，初始总电流 = "
                    << dock_initial_total_current
                    << " mA"
                    << std::endl;

                last_dock_mode = true;
            }

            if (!dock_mode_ && last_dock_mode) {

                // 对接 -> 自平衡
                // 重新创建纵向串级PID，清除历史积分与误差状态
                resetBalancePIDs();

                // 重置纵向控制相关滤波器
                targetAngleFilter =
                    std::make_unique<LowPassFilter>(
                        50.0, 1.0 / rate);

                speedOutputFilter =
                    std::make_unique<LowPassFilter>(
                        50.0, 1.0 / rate);

                // 清除纵向控制中间变量
                speed_angle_adjust = 0.0f;
                speed_angle_adjust_smooth = 0.0f;
                angleAdjust = 0.0f;
                target_gyro = 0.0f;
                forwardVal = 0.0f;

                angleTick = 0;
                speedTick = 0;

                dock_takeover = false;
                dock_initial_total_current = 0.0f;

                last_dock_mode = false;

                std::cout
                    << "[balance] 退出对接模式，"
                    << "纵向串级PID已重新初始化"
                    << std::endl;
            }

            // ========================================================
            // 模式控制
            // ========================================================

            if (dock_mode_)
            {
                // ====================================================
                // 对接模式
                //
                // 纵向串级PID：
                // speedPID、anglePID、gyroPID均不调用
                //
                // 横摆PID：
                // gyroZPID继续运行
                // ====================================================

                angleAdjust = 0.0f;
                speed_angle_adjust = 0.0f;
                speed_angle_adjust_smooth = 0.0f;

                target_gyro = 0.0f;
                forwardVal = 0.0f;

                target_angle = banlanceOffset;

                // ----------------------------------------------------
                // 1. 初始电流保持与油门接管
                // ----------------------------------------------------

                float initial_base_current =
                    dock_initial_total_current / 2.0f;

                // 初始保持电流超过单电机最大值时，按最大值处理
                float takeover_threshold =
                    std::min(
                        initial_base_current,
                        DOCK_MAX_CURRENT);

                // 油门映射：单电机最大6000mA
                float stick_current =
                    DOCK_MAX_CURRENT * dock_throttle_;

                // 首次超过切换前单轮电流阈值后，锁存油门控制
                // 阈值达到6000mA时，允许在触及最大值时接管
                if (!dock_takeover) {

                    if (stick_current > takeover_threshold ||
                        (takeover_threshold >= DOCK_MAX_CURRENT &&
                         stick_current >= DOCK_MAX_CURRENT)) {

                        dock_takeover = true;

                        std::cout
                            << "[dock] 油门已接管推进电流"
                            << std::endl;
                    }
                }

                float base_current = 0.0f;

                if (dock_takeover) {
                    base_current = stick_current;
                }
                else {
                    base_current = initial_base_current;
                }

                base_current = std::clamp(
                    base_current,
                    0.0f,
                    DOCK_MAX_CURRENT);

                // ----------------------------------------------------
                // 2. 档位方向控制
                // ----------------------------------------------------

                float direction =
                    static_cast<float>(gear_direction_);

                // 档位处于中位时不输出纵向推进电流
                float longitudinal_current =
                    direction * base_current;

                // ----------------------------------------------------
                // 3. 横摆角速度PID
                // ----------------------------------------------------

                yawAdjust =
                    -gyroZPID.GetControl(
                        target_yaw_rate_ - gyroZ,
                        1000.0 / rate);

                yawAdjust = AD::Utils::ClampAbs(
                    yawAdjust,
                    4000);

                // ----------------------------------------------------
                // 4. 根据横摆PID结果分配左右轮电流
                //
                // 左右轮目标电流：
                // IL = Ibase + Iyaw
                // IR = Ibase - Iyaw
                //
                // 先计算最终期望电流，再消除电机倍率补偿，
                // 避免对切换前已补偿的电流重复施加倍率。
                // ----------------------------------------------------

                float desired_left =
                    longitudinal_current + yawAdjust;

                float desired_right =
                    longitudinal_current - yawAdjust;

                // 保证左右电机最终输出均不超过6000mA
                float max_abs_current =
                    std::max(
                        std::abs(desired_left),
                        std::abs(desired_right));

                float current_scale = 1.0f;

                if (max_abs_current > DOCK_MAX_CURRENT) {

                    current_scale =
                        DOCK_MAX_CURRENT / max_abs_current;
                }

                desired_left *= current_scale;
                desired_right *= current_scale;

                // 抵消原有左右轮输出倍率，最终输出保持期望值
                left = desired_left / kleft;
                right = desired_right / kright;
            }
            else
            {
                // ====================================================
                // 正常自平衡模式
                // ====================================================

                double target_speed_timestamp =
                    this->now().seconds();

                target_speed_file << target_speed_ << std::endl;

                timestamp_target_speed_file
                    << std::fixed << std::setprecision(9)
                    << target_speed_timestamp << std::endl;

                // 速度PID输出滤波
                speed_angle_adjust_smooth =
                    speedOutputFilter->update(
                        speed_angle_adjust);

                angleAdjust = speed_angle_adjust_smooth;

                angleAdjust = AD::Utils::ClampAbs(
                    angleAdjust,
                    8.0);

                target_angle =
                    banlanceOffset + angleAdjust;

                target_angle =
                    targetAngleFilter->update(target_angle);

                double target_angle_timestamp =
                    this->now().seconds();

                target_angle_file << target_angle << std::endl;

                timestamp_target_angle_file
                    << std::fixed << std::setprecision(9)
                    << target_angle_timestamp << std::endl;

                // 角度环 100Hz
                angleTick++;

                if (angleTick >= 2) {

                    angleTick = 0;

                    target_gyro =
                        anglePID->GetControl(
                            target_angle - angle,
                            1000.0 / 100.0);
                }

                // 纵向角速度环 200Hz
                forwardVal =
                    -gyroPID->GetControl(
                        (target_gyro - gyro) * 1,
                        1000.0 / rate);

                forwardVal = AD::Utils::ClampAbs(
                    forwardVal,
                    15000);

                // 横摆角速度环 200Hz
                yawAdjust =
                    -gyroZPID.GetControl(
                        target_yaw_rate_ - gyroZ,
                        1000.0 / rate);

                yawAdjust = AD::Utils::ClampAbs(
                    yawAdjust,
                    4000);

                left = forwardVal + yawAdjust;
                right = forwardVal - yawAdjust;
            }

            // ========================================================
            // 数据记录
            // ========================================================

            double target_gyro_timestamp =
                this->now().seconds();

            target_gyro_file << target_gyro << std::endl;

            timestamp_target_gyro_file
                << std::fixed << std::setprecision(9)
                << target_gyro_timestamp << std::endl;

            double current_gyro_z_timestamp =
                this->now().seconds();

            gyro_z_file << gyroZ << std::endl;

            timestamp_gyro_z_file
                << std::fixed << std::setprecision(9)
                << current_gyro_z_timestamp << std::endl;

            double gyro_z_filtered_ts =
                this->now().seconds();

            gyro_z_filtered_file << gyroZ << std::endl;

            timestamp_gyro_z_filtered_file
                << std::fixed << std::setprecision(9)
                << gyro_z_filtered_ts << std::endl;

            double target_yaw_rate_ts =
                this->now().seconds();

            target_yaw_rate_file
                << target_yaw_rate_ << std::endl;

            timestamp_target_yaw_rate_file
                << std::fixed << std::setprecision(9)
                << target_yaw_rate_ts << std::endl;

            // ========================================================
            // 电机输出
            // ========================================================

            if (dock_mode_) {

                // 对接模式：最终电流硬限幅为±6000mA
                left = std::clamp(
                    kleft * left,
                    -DOCK_MAX_CURRENT,
                    DOCK_MAX_CURRENT);

                right = std::clamp(
                    kright * right,
                    -DOCK_MAX_CURRENT,
                    DOCK_MAX_CURRENT);
            }
            else {

                // 自平衡模式保留原有限幅与倍率补偿
                left = kleft * WheelLimiter.clamp(left);
                right = kright * WheelLimiter.clamp(right);
            }

            motorLeft.SetCurrent(left);
            motorRight.SetCurrent(right);

            // 保存实际发送电流，供下一次进入对接模式时使用
            last_applied_left = left;
            last_applied_right = right;

            heart_beat_tick++;

            if (heart_beat_tick >= 10) {

                heart_beat_tick = 0;

                motorLeft.SendHeartBeat();
                motorRight.SendHeartBeat();
            }

            double current_ts =
                this->now().seconds();

            left_current_file << left << std::endl;

            timestamp_left_current_file
                << std::fixed << std::setprecision(9)
                << current_ts << std::endl;

            right_current_file << right << std::endl;

            timestamp_right_current_file
                << std::fixed << std::setprecision(9)
                << current_ts << std::endl;

            // ========================================================
            // ROS状态发布
            // ========================================================

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

        // ============================================================
        // 文件关闭
        // ============================================================

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

    auto controller =
        std::make_shared<BalanceController>();

    controller->run();

    rclcpp::shutdown();

    return 0;
}

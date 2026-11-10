// ================================================================
//  Dock Motor Controller -- RC hold-to-run version
//
//  订阅:  /rc_input  (sensor_msgs/msg/Joy)
//
//  行为:
//    - 按住 "tighten" 按钮 -> 两路电机 CW 全速转
//    - 按住 "release" 按钮 -> 两路电机 CCW 全速转
//    - 松开 / 两个键同时按 -> 立刻停
//    - 超过 watchdog_timeout_ms 没收到 Joy 消息 -> 立刻停 (失联保护)
//
//  按钮索引通过 ROS 参数配置:
//    tighten_button_index   (默认 4)
//    release_button_index   (默认 5)
//
//  查按键索引:
//    ros2 topic echo /rc_input
//    按住某个开关, 看 buttons[] 哪个变成 1, 那就是 index
// ================================================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <gpiod.h>

#include <errno.h>
#include <string.h>
#include <unistd.h>   // access()

using namespace std::chrono_literals;

// ================================================================
//  Dock Motor Controller
// ================================================================
class DockMotorController : public rclcpp::Node
{
public:
    DockMotorController()
        : Node("dock_motor_controller")
    {
        // --------------------------------------------------------
        // ROS 参数: 按钮索引 / 看门狗
        // --------------------------------------------------------
        tighten_btn_ =
            this->declare_parameter<int>("tighten_button_index", 4);
        release_btn_ =
            this->declare_parameter<int>("release_button_index", 5);
        watchdog_timeout_ms_ =
            this->declare_parameter<int>("watchdog_timeout_ms", 500);

        RCLCPP_INFO(
            this->get_logger(),
            "Initializing dock motor controller (RC hold-to-run)...");

        // ========================================================
        // 硬件映射 (和原版一致)
        //
        // Motor1
        //   Speed    : Pin 32 -> PWM7 -> pwmchip3/pwm0
        //   Direction: Pin 15 -> PWM1 -> pwmchip0/pwm0
        //
        // Motor2
        //   Speed    : Pin 33 -> PWM5 -> pwmchip2/pwm0
        //   Direction: Pin 7  -> GPIO09 -> gpiochip0 line 144
        // ========================================================
        speed_pwm1_path_ = "/sys/class/pwm/pwmchip3/pwm0";
        dir_pwm1_path_   = "/sys/class/pwm/pwmchip0/pwm0";
        speed_pwm2_path_ = "/sys/class/pwm/pwmchip2/pwm0";

        // --------------------------------------------------------
        // 初始化 PWM / GPIO
        // --------------------------------------------------------
        if (!initPWM(speed_pwm1_path_))
            throw std::runtime_error("Motor1 speed PWM init failed");
        if (!initPWM(dir_pwm1_path_))
            throw std::runtime_error("Motor1 dir PWM init failed");
        if (!initPWM(speed_pwm2_path_))
            throw std::runtime_error("Motor2 speed PWM init failed");
        if (!initGPIO())
            throw std::runtime_error("Motor2 dir GPIO init failed");

        // 上电默认停止, 方向先设 CCW (无害)
        stopMotors();
        setDirectionCCW();

        // --------------------------------------------------------
        // 订阅遥控器 /rc_input
        // --------------------------------------------------------
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/rc_input_b1",
            10,
            std::bind(
                &DockMotorController::joyCallback,
                this,
                std::placeholders::_1));

        // --------------------------------------------------------
        // 看门狗定时器: 50 Hz 周期, 检查失联 + 刷新电机状态
        // --------------------------------------------------------
        watchdog_timer_ = this->create_wall_timer(
            20ms,
            std::bind(&DockMotorController::watchdogTick, this));

        last_rc_time_ = this->now();

        // --------------------------------------------------------
        // 启动日志
        // --------------------------------------------------------
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "Dock Motor Controller (RC hold) started");
        RCLCPP_INFO(this->get_logger(), "Subscribed topic : /rc_input (sensor_msgs/Joy)");
        RCLCPP_INFO(this->get_logger(), "Tighten button   : buttons[%d]  (CW)", tighten_btn_);
        RCLCPP_INFO(this->get_logger(), "Release button   : buttons[%d]  (CCW)", release_btn_);
        RCLCPP_INFO(this->get_logger(), "Watchdog timeout : %d ms", watchdog_timeout_ms_);
        RCLCPP_INFO(this->get_logger(), "PWM period       : %d ns", kPeriodNs);
        RCLCPP_INFO(this->get_logger(), "STOP duty        : %d ns", kStopDutyNs);
        RCLCPP_INFO(this->get_logger(), "MAX duty         : %d ns", kMaxRunDutyNs);
        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Find button index:");
        RCLCPP_INFO(this->get_logger(), "  ros2 topic echo /rc_input");
        RCLCPP_INFO(this->get_logger(), "  hold a switch, watch buttons[] which turns to 1");
        RCLCPP_INFO(this->get_logger(), "==========================================");
    }

    ~DockMotorController() override
    {
        stopMotors();
        if (gpio_line_ != nullptr) { gpiod_line_release(gpio_line_); gpio_line_ = nullptr; }
        if (gpio_chip_ != nullptr) { gpiod_chip_close(gpio_chip_); gpio_chip_ = nullptr; }
    }

private:
    // ============================================================
    // PWM 参数 (和原版一致)
    // ============================================================
    static constexpr int kPeriodNs     = 50000;   // 20 kHz
    static constexpr int kStopDutyNs   = 50000;   // 100% HIGH -> 停 (反逻辑驱动板)
    static constexpr int kMaxRunDutyNs = 10;       // ~0% HIGH  -> 全速

    static constexpr unsigned int kMotor2DirLine = 144;

    // ============================================================
    // GPIO / PWM paths
    // ============================================================
    gpiod_chip* gpio_chip_{nullptr};
    gpiod_line* gpio_line_{nullptr};

    std::string speed_pwm1_path_;
    std::string dir_pwm1_path_;
    std::string speed_pwm2_path_;

    // ============================================================
    // ROS
    // ============================================================
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr                           watchdog_timer_;

    int            tighten_btn_        = 4;
    int            release_btn_        = 5;
    int            watchdog_timeout_ms_ = 500;

    // ============================================================
    // 最新遥控器状态 (被 Joy 回调写, 被看门狗定时器读)
    // ============================================================
    std::mutex rc_mutex_;
    bool       tighten_pressed_{false};
    bool       release_pressed_{false};
    rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
    bool       have_rc_{false};

    // 当前已经输出的动作: 0=stop, 1=CW, 2=CCW
    int applied_action_{0};

    // ============================================================
    // sysfs 写入
    // ============================================================
    bool writeFile(const std::string& path, const std::string& value)
    {
        std::string command =
            "printf '%s\\n' '" + value + "' | sudo -n tee '" + path + "' > /dev/null";
        const int ret = std::system(command.c_str());
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "write fail: %s = %s, ret=%d",
                         path.c_str(), value.c_str(), ret);
            return false;
        }
        return true;
    }

    // ============================================================
    // PWM 初始化
    // ============================================================
    bool initPWM(const std::string& pwm_path)
    {
        const std::string pwm_name = "/pwm0";
        const auto pos = pwm_path.rfind(pwm_name);
        if (pos == std::string::npos) {
            RCLCPP_ERROR(this->get_logger(), "bad PWM path: %s", pwm_path.c_str());
            return false;
        }
        const std::string chip_path = pwm_path.substr(0, pos);

        // 检查是否已 export
        if (access((pwm_path + "/period").c_str(), F_OK) != 0) {
            RCLCPP_INFO(this->get_logger(), "exporting PWM: %s", pwm_path.c_str());
            std::string cmd =
                "printf '0\\n' | sudo -n tee '" + chip_path + "/export' > /dev/null";
            if (std::system(cmd.c_str()) != 0) {
                RCLCPP_ERROR(this->get_logger(), "export failed: %s", pwm_path.c_str());
                return false;
            }
            std::this_thread::sleep_for(200ms);
        }

        if (!writeFile(pwm_path + "/period",     std::to_string(kPeriodNs)))   return false;
        if (!writeFile(pwm_path + "/duty_cycle", std::to_string(kStopDutyNs))) return false;
        if (!writeFile(pwm_path + "/enable",    "1"))                         return false;

        RCLCPP_INFO(this->get_logger(), "PWM ok: %s", pwm_path.c_str());
        return true;
    }

    // ============================================================
    // GPIO 初始化
    // ============================================================
    bool initGPIO()
    {
        gpio_chip_ = gpiod_chip_open("/dev/gpiochip0");
        if (!gpio_chip_) {
            RCLCPP_ERROR(this->get_logger(), "open /dev/gpiochip0 failed");
            return false;
        }
        gpio_line_ = gpiod_chip_get_line(gpio_chip_, kMotor2DirLine);
        if (!gpio_line_) {
            RCLCPP_ERROR(this->get_logger(), "get line %u failed", kMotor2DirLine);
            return false;
        }
        const int ret = gpiod_line_request_output(gpio_line_, "dock_motor_controller", 0);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "request line %u failed: ret=%d errno=%d (%s)",
                         kMotor2DirLine, ret, errno, strerror(errno));
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "GPIO ok: gpiochip0 line %u", kMotor2DirLine);
        return true;
    }

    bool setGPIOValue(int v)
    {
        if (!gpio_line_) return false;
        if (gpiod_line_set_value(gpio_line_, v ? 1 : 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "set GPIO %u failed", kMotor2DirLine);
            return false;
        }
        return true;
    }

    // ============================================================
    // 电机动作
    // ============================================================
    void runMotorsAtMaxSpeed()
    {
        writeFile(speed_pwm1_path_ + "/duty_cycle", std::to_string(kMaxRunDutyNs));
        writeFile(speed_pwm2_path_ + "/duty_cycle", std::to_string(kMaxRunDutyNs));
    }

    void stopMotors()
    {
        writeFile(speed_pwm1_path_ + "/duty_cycle", std::to_string(kStopDutyNs));
        writeFile(speed_pwm2_path_ + "/duty_cycle", std::to_string(kStopDutyNs));
    }

    void setDirectionCW()
    {
        writeFile(dir_pwm1_path_ + "/duty_cycle", "0");       // Motor1 LOW
        setGPIOValue(0);                                       // Motor2 LOW
    }

    void setDirectionCCW()
    {
        writeFile(dir_pwm1_path_ + "/duty_cycle", std::to_string(kPeriodNs)); // Motor1 HIGH
        setGPIOValue(1);                                                       // Motor2 HIGH
    }

    // ============================================================
    // Joy 回调: 只更新状态, 不动电机
    // ============================================================
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(rc_mutex_);
        last_rc_time_ = this->now();
        have_rc_      = true;

        tighten_pressed_ =
            (tighten_btn_ >= 0 && tighten_btn_ < (int)msg->buttons.size())
            ? (msg->buttons[tighten_btn_] != 0) : false;

        release_pressed_ =
            (release_btn_ >= 0 && release_btn_ < (int)msg->buttons.size())
            ? (msg->buttons[release_btn_] != 0) : false;
    }

    // ============================================================
    // 看门狗 / 控制刷新 (50 Hz)
    // ============================================================
    void watchdogTick()
    {
        bool t = false, r = false;
        bool lost = false;

        {
            std::lock_guard<std::mutex> lock(rc_mutex_);
            t = tighten_pressed_;
            r = release_pressed_;

            if (have_rc_) {
                const double dt_ms =
                    (this->now() - last_rc_time_).seconds() * 1000.0;
                if (dt_ms > watchdog_timeout_ms_) {
                    lost = true;
                }
            }
        }

        int desired = 0;
        if (!lost) {
            // 互斥: 单键按下才动, 双键同按 / 都不按 -> 停
            if (t && !r)      desired = 1;  // CW = 拧紧
            else if (r && !t) desired = 2;  // CCW = 松开
            else              desired = 0;
        } else {
            desired = 0;
        }

        applyAction(desired, lost);
    }

    // ============================================================
    // 根据目标动作驱动硬件, 带边沿切换保护
    // ============================================================
    void applyAction(int desired, bool lost)
    {
        if (desired == applied_action_) {
            // 状态没变, 啥也不做 (避免每次都写 sysfs)
            return;
        }

        const int prev = applied_action_;

        // 正在转 -> 要换方向 / 要停: 先刹一下
        if (prev != 0) {
            stopMotors();
            std::this_thread::sleep_for(100ms);
        }

        if (desired == 1) {
            setDirectionCW();
            std::this_thread::sleep_for(50ms);  // 方向电平稳定
            runMotorsAtMaxSpeed();
            RCLCPP_INFO(this->get_logger(), "MOTOR RUN  CW (tighten)");
        } else if (desired == 2) {
            setDirectionCCW();
            std::this_thread::sleep_for(50ms);
            runMotorsAtMaxSpeed();
            RCLCPP_INFO(this->get_logger(), "MOTOR RUN  CCW (release)");
        } else {
            stopMotors();
            if (lost) {
                RCLCPP_WARN(this->get_logger(),
                            "MOTOR STOPPED (RC watchdog trip)");
            } else {
                RCLCPP_INFO(this->get_logger(), "MOTOR STOP");
            }
        }

        applied_action_ = desired;
    }
};

// ================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<DockMotorController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        fprintf(stderr, "Fatal: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <gpiod.h>

// ============================================================
// Linux SocketCAN
// ============================================================

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std::chrono_literals;


// ================================================================
// Dock Motor Controller
// ================================================================

class DockMotorController : public rclcpp::Node
{
public:

    DockMotorController()
        : Node("dock_motor_controller")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Initializing dual docking motor controller...");

        // ========================================================
        // 硬件映射
        // ========================================================
        //
        // Motor 1
        //   Speed    : Pin 32 -> PWM7 -> pwmchip3/pwm0
        //   Direction: Pin 15 -> PWM1 -> pwmchip0/pwm0
        //
        // Motor 2
        //   Speed    : Pin 33 -> PWM5 -> pwmchip2/pwm0
        //   Direction: Pin 7  -> GPIO09 -> gpiochip0 line 144
        //
        // ========================================================

        speed_pwm1_path_ =
            "/sys/class/pwm/pwmchip3/pwm0";

        dir_pwm1_path_ =
            "/sys/class/pwm/pwmchip0/pwm0";

        speed_pwm2_path_ =
            "/sys/class/pwm/pwmchip2/pwm0";


        // ========================================================
        // 初始化 PWM
        // ========================================================

        if (!initPWM(speed_pwm1_path_))
        {
            throw std::runtime_error(
                "Motor1 speed PWM initialization failed");
        }

        if (!initPWM(dir_pwm1_path_))
        {
            throw std::runtime_error(
                "Motor1 direction PWM initialization failed");
        }

        if (!initPWM(speed_pwm2_path_))
        {
            throw std::runtime_error(
                "Motor2 speed PWM initialization failed");
        }


        // ========================================================
        // 初始化 GPIO
        // ========================================================

        if (!initGPIO())
        {
            throw std::runtime_error(
                "Motor2 direction GPIO initialization failed");
        }


        // ========================================================
        // 上电默认停止
        // ========================================================

        stopMotors();

        setDirectionCCW();


        // ========================================================
        // ROS2 subscriber
        // ========================================================

        command_sub_ =
            this->create_subscription<std_msgs::msg::String>(
                "/dock_motor/command",
                10,
                std::bind(
                    &DockMotorController::commandCallback,
                    this,
                    std::placeholders::_1));


        // ========================================================
        // 打开 CAN1
        // ========================================================

        if (!initCAN())
        {
            throw std::runtime_error(
                "CAN1 initialization failed");
        }


        // ========================================================
        // 启动 CAN 接收线程
        // ========================================================

        can_running_ = true;

        can_thread_ =
            std::thread(
                &DockMotorController::canReceiveLoop,
                this);


        // ========================================================
        // 启动电机控制线程
        // ========================================================

        motor_thread_ =
            std::thread(
                &DockMotorController::motorControlLoop,
                this);


        // ========================================================
        // 打印配置
        // ========================================================

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "Dual Dock Motor Controller started");

        RCLCPP_INFO(
            this->get_logger(),
            "Motor1 speed : PWM7 -> pwmchip3/pwm0");

        RCLCPP_INFO(
            this->get_logger(),
            "Motor1 dir   : PWM1 -> pwmchip0/pwm0");

        RCLCPP_INFO(
            this->get_logger(),
            "Motor2 speed : PWM5 -> pwmchip2/pwm0");

        RCLCPP_INFO(
            this->get_logger(),
            "Motor2 dir   : GPIO09 -> gpiochip0 line 144");

        RCLCPP_INFO(
            this->get_logger(),
            "PWM period   : %d ns",
            kPeriodNs);

        RCLCPP_INFO(
            this->get_logger(),
            "STOP duty    : %d ns",
            kStopDutyNs);

        RCLCPP_INFO(
            this->get_logger(),
            "MAX duty     : %d ns",
            kMaxRunDutyNs);

        RCLCPP_INFO(
            this->get_logger(),
            "CAN interface: can1");

        RCLCPP_INFO(
            this->get_logger(),
            "CAN ID       : 0x%03X",
            kCanCommandId);

        RCLCPP_INFO(
            this->get_logger(),
            "Tighten cmd  : 0x%02X -> 3.5 s",
            kCanTightenCmd);

        RCLCPP_INFO(
            this->get_logger(),
            "Release cmd  : 0x%02X -> 4.0 s",
            kCanReleaseCmd);

        RCLCPP_INFO(
            this->get_logger(),
            "ROS commands:");

        RCLCPP_INFO(
            this->get_logger(),
            "  run_cw");

        RCLCPP_INFO(
            this->get_logger(),
            "  run_ccw");

        RCLCPP_INFO(
            this->get_logger(),
            "  stop");

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");
    }


    // ============================================================
    // 析构
    // ============================================================

    ~DockMotorController()
    {
        // 停止 CAN 线程
        can_running_ = false;

        // 唤醒 motor thread
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command_cv_.notify_all();
        }

        // 关闭 CAN socket
        if (can_socket_ >= 0)
        {
            shutdown(
                can_socket_,
                SHUT_RDWR);

            close(can_socket_);

            can_socket_ = -1;
        }

        // 等待 CAN 线程退出
        if (can_thread_.joinable())
        {
            can_thread_.join();
        }

        // 等待 motor thread 退出
        if (motor_thread_.joinable())
        {
            motor_thread_.join();
        }

        // 强制停止
        stopMotors();

        // 释放 GPIO
        if (gpio_line_ != nullptr)
        {
            gpiod_line_release(gpio_line_);
            gpio_line_ = nullptr;
        }

        if (gpio_chip_ != nullptr)
        {
            gpiod_chip_close(gpio_chip_);
            gpio_chip_ = nullptr;
        }
    }


private:

    // ============================================================
    // PWM 参数
    // ============================================================

    static constexpr int kPeriodNs = 50000;

    static constexpr int kStopDutyNs = 50000;

    // 你现在实测最高速
    static constexpr int kMaxRunDutyNs = 10;


    // ============================================================
    // Motor2 GPIO
    // ============================================================

    static constexpr unsigned int kMotor2DirLine = 144;


    // ============================================================
    // CAN 参数
    // ============================================================

    // ========================================================
    // 这里改成你的实际 CAN ID
    // ========================================================

    static constexpr canid_t kCanCommandId = 0x123;


    // ========================================================
    // CAN 数据
    //
    // data[0] = 0x01 -> 拧紧
    // data[0] = 0x02 -> 松开
    // ========================================================

    static constexpr uint8_t kCanTightenCmd = 0x01;

    static constexpr uint8_t kCanReleaseCmd = 0x02;


    // ============================================================
    // 电机动作
    // ============================================================

    enum class MotorAction
    {
        NONE,
        TIGHTEN,
        RELEASE
    };


    // ============================================================
    // GPIO
    // ============================================================

    gpiod_chip* gpio_chip_{nullptr};

    gpiod_line* gpio_line_{nullptr};


    // ============================================================
    // PWM
    // ============================================================

    std::string speed_pwm1_path_;

    std::string dir_pwm1_path_;

    std::string speed_pwm2_path_;


    // ============================================================
    // ROS2
    // ============================================================

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        command_sub_;


    // ============================================================
    // CAN
    // ============================================================

    int can_socket_{-1};

    std::atomic<bool> can_running_{false};

    std::thread can_thread_;


    // ============================================================
    // Motor Control Thread
    // ============================================================

    std::thread motor_thread_;

    std::mutex command_mutex_;

    std::condition_variable command_cv_;

    MotorAction requested_action_{MotorAction::NONE};


    // ============================================================
    // sysfs 写入
    // ============================================================

    bool writeFile(
        const std::string& path,
        const std::string& value)
    {
        std::string command =
            "printf '%s\\n' '" +
            value +
            "' | sudo -n tee '" +
            path +
            "' > /dev/null";

        const int ret =
            std::system(command.c_str());

        if (ret != 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to write: %s = %s, ret=%d",
                path.c_str(),
                value.c_str(),
                ret);

            return false;
        }

        return true;
    }


    // ============================================================
    // 初始化 PWM
    // ============================================================

    bool initPWM(
        const std::string& pwm_path)
    {
        const std::string pwm_name = "/pwm0";

        const auto pos =
            pwm_path.rfind(pwm_name);

        if (pos == std::string::npos)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid PWM path: %s",
                pwm_path.c_str());

            return false;
        }

        const std::string chip_path =
            pwm_path.substr(0, pos);


        // --------------------------------------------------------
        // 检查是否已经 export
        // --------------------------------------------------------

        {
            std::ifstream test(
                pwm_path + "/period");

            if (!test.good())
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Exporting PWM: %s",
                    pwm_path.c_str());

                std::string command =
                    "printf '0\\n' | sudo -n tee '" +
                    chip_path +
                    "/export' > /dev/null";

                const int ret =
                    std::system(command.c_str());

                if (ret != 0)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to export PWM: %s",
                        pwm_path.c_str());

                    return false;
                }

                std::this_thread::sleep_for(200ms);
            }
        }


        // --------------------------------------------------------
        // 设置周期
        // --------------------------------------------------------

        if (!writeFile(
                pwm_path + "/period",
                std::to_string(kPeriodNs)))
        {
            return false;
        }


        // --------------------------------------------------------
        // 初始停止
        // --------------------------------------------------------

        if (!writeFile(
                pwm_path + "/duty_cycle",
                std::to_string(kStopDutyNs)))
        {
            return false;
        }


        // --------------------------------------------------------
        // Enable
        // --------------------------------------------------------

        if (!writeFile(
                pwm_path + "/enable",
                "1"))
        {
            return false;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "PWM initialized: %s",
            pwm_path.c_str());

        return true;
    }


    // ============================================================
    // 初始化 GPIO
    // ============================================================

    bool initGPIO()
    {
        gpio_chip_ =
            gpiod_chip_open(
                "/dev/gpiochip0");

        if (gpio_chip_ == nullptr)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open /dev/gpiochip0");

            return false;
        }


        gpio_line_ =
            gpiod_chip_get_line(
                gpio_chip_,
                kMotor2DirLine);

        if (gpio_line_ == nullptr)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to get GPIO line %u",
                kMotor2DirLine);

            return false;
        }


        const int ret =
            gpiod_line_request_output(
                gpio_line_,
                "dock_motor_controller",
                0);

        if (ret < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to request GPIO line %u",
                kMotor2DirLine);

            return false;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "GPIO initialized: gpiochip0 line %u",
            kMotor2DirLine);

        return true;
    }


    // ============================================================
    // GPIO 设置
    // ============================================================

    bool setGPIOValue(
        int value)
    {
        if (gpio_line_ == nullptr)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "GPIO line is not initialized");

            return false;
        }


        const int ret =
            gpiod_line_set_value(
                gpio_line_,
                value ? 1 : 0);

        if (ret < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to set GPIO line %u",
                kMotor2DirLine);

            return false;
        }

        return true;
    }


    // ============================================================
    // 两个电机最高速
    // ============================================================

    void runMotorsAtMaxSpeed()
    {
        const bool ok1 =
            writeFile(
                speed_pwm1_path_ +
                    "/duty_cycle",
                std::to_string(
                    kMaxRunDutyNs));

        const bool ok2 =
            writeFile(
                speed_pwm2_path_ +
                    "/duty_cycle",
                std::to_string(
                    kMaxRunDutyNs));


        if (!ok1 || !ok2)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to start motors at maximum speed");

            return;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "Both docking motors RUN at MAX speed");
    }


    // ============================================================
    // 停止
    // ============================================================

    void stopMotors()
    {
        writeFile(
            speed_pwm1_path_ +
                "/duty_cycle",
            std::to_string(
                kStopDutyNs));

        writeFile(
            speed_pwm2_path_ +
                "/duty_cycle",
            std::to_string(
                kStopDutyNs));

        RCLCPP_INFO(
            this->get_logger(),
            "Both docking motors STOP");
    }


    // ============================================================
    // CW
    // ============================================================

    void setDirectionCW()
    {
        // Motor1:
        // LOW -> CW

        const bool ok1 =
            writeFile(
                dir_pwm1_path_ +
                    "/duty_cycle",
                "0");


        // Motor2:
        // LOW -> CW

        const bool ok2 =
            setGPIOValue(0);


        if (ok1 && ok2)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Both docking motors direction = CW");
        }
    }


    // ============================================================
    // CCW
    // ============================================================

    void setDirectionCCW()
    {
        // Motor1:
        // HIGH -> CCW

        const bool ok1 =
            writeFile(
                dir_pwm1_path_ +
                    "/duty_cycle",
                std::to_string(
                    kPeriodNs));


        // Motor2:
        // HIGH -> CCW

        const bool ok2 =
            setGPIOValue(1);


        if (ok1 && ok2)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Both docking motors direction = CCW");
        }
    }


    // ============================================================
    // 请求新的动作
    // ============================================================

    void requestAction(
        MotorAction action)
    {
        {
            std::lock_guard<std::mutex> lock(
                command_mutex_);

            requested_action_ = action;
        }

        command_cv_.notify_one();
    }


    // ============================================================
    // 检查当前是否有新的命令
    // ============================================================

    bool hasNewCommand()
    {
        std::lock_guard<std::mutex> lock(
            command_mutex_);

        return requested_action_ !=
               MotorAction::NONE;
    }


    // ============================================================
    // 取出新命令
    // ============================================================

    MotorAction takeRequestedAction()
    {
        std::lock_guard<std::mutex> lock(
            command_mutex_);

        MotorAction action =
            requested_action_;

        requested_action_ =
            MotorAction::NONE;

        return action;
    }


    // ============================================================
    // 电机控制线程
    // ============================================================

    void motorControlLoop()
    {
        while (rclcpp::ok())
        {
            MotorAction action;

            // ----------------------------------------------------
            // 等待新动作
            // ----------------------------------------------------

            {
                std::unique_lock<std::mutex> lock(
                    command_mutex_);

                command_cv_.wait(
                    lock,
                    [this]()
                    {
                        return
                            requested_action_ !=
                                MotorAction::NONE
                            ||
                            !rclcpp::ok();
                    });

                if (!rclcpp::ok())
                {
                    break;
                }

                action =
                    requested_action_;

                requested_action_ =
                    MotorAction::NONE;
            }


            // ----------------------------------------------------
            // 先停止
            // ----------------------------------------------------

            stopMotors();

            std::this_thread::sleep_for(
                200ms);


            // ----------------------------------------------------
            // 判断方向
            // ----------------------------------------------------

            double run_time = 0.0;


            if (action == MotorAction::TIGHTEN)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "CAN command: TIGHTEN");

                setDirectionCW();

                run_time = 3.5;
            }
            else if (action == MotorAction::RELEASE)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "CAN command: RELEASE");

                setDirectionCCW();

                run_time = 4.0;
            }
            else
            {
                continue;
            }


            // ----------------------------------------------------
            // 等待方向稳定
            // ----------------------------------------------------

            std::this_thread::sleep_for(
                300ms);


            // ----------------------------------------------------
            // 启动电机
            // ----------------------------------------------------

            runMotorsAtMaxSpeed();


            // ----------------------------------------------------
            // 计时
            //
            // 每 10 ms 检查一次有没有新的 CAN 命令
            // ----------------------------------------------------

            const auto start =
                std::chrono::steady_clock::now();

            bool interrupted = false;

            while (true)
            {
                if (hasNewCommand())
                {
                    interrupted = true;
                    break;
                }


                const auto now =
                    std::chrono::steady_clock::now();

                const double elapsed =
                    std::chrono::duration<double>(
                        now - start).count();


                if (elapsed >= run_time)
                {
                    break;
                }


                std::this_thread::sleep_for(
                    10ms);
            }


            // ----------------------------------------------------
            // 停止
            // ----------------------------------------------------

            stopMotors();


            if (interrupted)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Motor action interrupted by new command");
            }
            else
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Motor action finished automatically");
            }
        }


        stopMotors();
    }


    // ============================================================
    // 初始化 CAN1
    // ============================================================

    bool initCAN()
    {
        // --------------------------------------------------------
        // 创建 RAW CAN socket
        // --------------------------------------------------------

        can_socket_ =
            socket(
                PF_CAN,
                SOCK_RAW,
                CAN_RAW);

        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to create CAN socket");

            return false;
        }


        // --------------------------------------------------------
        // 获取 can1 接口索引
        // --------------------------------------------------------

        struct ifreq ifr {};

        std::snprintf(
            ifr.ifr_name,
            IFNAMSIZ,
            "can1");


        if (ioctl(
                can_socket_,
                SIOCGIFINDEX,
                &ifr) < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to get CAN1 interface index");

            close(can_socket_);
            can_socket_ = -1;

            return false;
        }


        // --------------------------------------------------------
        // 绑定 can1
        // --------------------------------------------------------

        struct sockaddr_can addr {};

        addr.can_family =
            AF_CAN;

        addr.can_ifindex =
            ifr.ifr_ifindex;


        if (bind(
                can_socket_,
                reinterpret_cast<struct sockaddr*>(
                    &addr),
                sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to bind CAN1");

            close(can_socket_);
            can_socket_ = -1;

            return false;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "CAN1 initialized successfully");

        return true;
    }


    // ============================================================
    // CAN 接收线程
    // ============================================================

    void canReceiveLoop()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "CAN1 receive thread started");


        while (can_running_)
        {
            struct can_frame frame {};

            const int nbytes =
                read(
                    can_socket_,
                    &frame,
                    sizeof(frame));


            if (!can_running_)
            {
                break;
            }


            if (nbytes < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                if (!can_running_)
                {
                    break;
                }

                RCLCPP_ERROR(
                    this->get_logger(),
                    "CAN1 read error");

                std::this_thread::sleep_for(
                    100ms);

                continue;
            }


            if (nbytes != sizeof(frame))
            {
                continue;
            }


            // ----------------------------------------------------
            // 打印收到的 CAN 帧
            // ----------------------------------------------------

            RCLCPP_DEBUG(
                this->get_logger(),
                "CAN1 RX ID=0x%03X DLC=%d DATA[0]=0x%02X",
                frame.can_id & CAN_SFF_MASK,
                frame.can_dlc,
                frame.can_dlc > 0
                    ? frame.data[0]
                    : 0);


            // ----------------------------------------------------
            // 检查 CAN ID
            // ----------------------------------------------------

            const canid_t rx_id =
                frame.can_id &
                CAN_SFF_MASK;


            if (rx_id != kCanCommandId)
            {
                continue;
            }


            // ----------------------------------------------------
            // 至少需要一个数据字节
            // ----------------------------------------------------

            if (frame.can_dlc < 1)
            {
                continue;
            }


            // ----------------------------------------------------
            // 判断命令
            // ----------------------------------------------------

            const uint8_t cmd =
                frame.data[0];


            // ====================================================
            // 拧紧
            // ====================================================

            if (cmd == kCanTightenCmd)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "CAN1: TIGHTEN command received");

                requestAction(
                    MotorAction::TIGHTEN);

                continue;
            }


            // ====================================================
            // 松开
            // ====================================================

            if (cmd == kCanReleaseCmd)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "CAN1: RELEASE command received");

                requestAction(
                    MotorAction::RELEASE);

                continue;
            }
        }


        RCLCPP_INFO(
            this->get_logger(),
            "CAN1 receive thread stopped");
    }


    // ============================================================
    // ROS2 命令
    // ============================================================

    void commandCallback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string cmd =
            msg->data;


        RCLCPP_INFO(
            this->get_logger(),
            "Received ROS command: %s",
            cmd.c_str());


        // --------------------------------------------------------
        // run_cw
        //
        // 为了保持你之前的测试方式，
        // ROS 的 run_cw 也执行固定 3.5 秒
        // --------------------------------------------------------

        if (cmd == "run_cw")
        {
            requestAction(
                MotorAction::TIGHTEN);

            return;
        }


        // --------------------------------------------------------
        // run_ccw
        //
        // ROS 的 run_ccw 执行固定 4 秒
        // --------------------------------------------------------

        if (cmd == "run_ccw")
        {
            requestAction(
                MotorAction::RELEASE);

            return;
        }


        // --------------------------------------------------------
        // stop
        // --------------------------------------------------------

        if (cmd == "stop")
        {
            requestAction(
                MotorAction::NONE);

            stopMotors();

            return;
        }


        // --------------------------------------------------------
        // 兼容旧 speed 命令
        // --------------------------------------------------------

        if (cmd.rfind("speed:", 0) == 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "speed command ignored, use fixed MAX speed");

            return;
        }


        RCLCPP_WARN(
            this->get_logger(),
            "Unknown ROS command: %s",
            cmd.c_str());
    }
};


// ================================================================
// main
// ================================================================

int main(
    int argc,
    char* argv[])
{
    rclcpp::init(
        argc,
        argv);


    try
    {
        auto node =
            std::make_shared<DockMotorController>();


        rclcpp::spin(
            node);
    }
    catch (const std::exception& e)
    {
        fprintf(
            stderr,
            "Fatal error: %s\n",
            e.what());

        rclcpp::shutdown();

        return 1;
    }


    rclcpp::shutdown();

    return 0;
}

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <trackmsg/msg/balance_date.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class DockingControlNode : public rclcpp::Node
{
public:
    struct Pose2
    {
        double x{0.0};
        double y{0.0};
        double theta{0.0};
    };

    struct PathPoint
    {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
        double curvature{0.0};
    };

    struct QuinticResult
    {
        double p{0.0};
        double dp{0.0};
        double ddp{0.0};
    };

    enum class ControlMode
    {
        QUINTIC_FAR,
        DIRECT_TARGET_NEAR
    };

    DockingControlNode()
        : Node("docking_control_node")
    {
        // ============================================================
        // 坐标定义
        // /docking/relative_pose 必须定义为：
        // x > 0 : 目标车在跟随车前方
        // y > 0 : 目标车在跟随车左侧
        // theta > 0 : 目标车相对跟随车左转
        // ============================================================

        // ============================================================
        // Stanley / 轨迹跟踪参数：保留当前仿真版本
        // ============================================================
        stanley_gain_ = 1.20;
        stanley_virtual_length_ = 0.80;
        lookahead_distance_ = 0.50;

        control_frequency_ = 50.0;
        replan_interval_ = 0.10;

        // ============================================================
        // 速度/角速度限制：结合实车接口使用
        // ============================================================
        max_command_speed_ = 1.35;
        max_yaw_rate_rad_ = 1.20;
        max_acceleration_ = 1.00;

        // ============================================================
        // 速度规划
        // ============================================================
        distance_gain_ = 0.40;
        speed_filter_alpha_ = 0.20;
        near_max_speed_boost_ = 0.25;
        catch_down_gain_ = 0.80;

        // ============================================================
        // 远近混合控制
        // distance > 3.0 m : 五次样条
        // distance <= 2.5 m: 直接动态目标跟踪
        // 2.5 ~ 3.0 m      : 保持当前模式
        // ============================================================
        direct_enter_distance_ = 2.50;
        quintic_enter_distance_ = 3.00;

        // ============================================================
        // 目标角速度前馈
        // target yaw rate = d(theta_relative)/dt + follower yaw rate
        // ============================================================
        target_yaw_rate_ff_gain_ = 0.80;
        max_target_yaw_rate_rad_ = 1.20;
        target_yaw_rate_filter_alpha_ = 0.30;

        // ============================================================
        // 五次样条过冲保护：保留当前仿真版本
        // ============================================================
        quintic_tangent_gain_ = 0.30;
        quintic_tangent_min_ = 0.10;
        quintic_tangent_max_ = 1.20;
        max_path_length_ratio_ = 1.70;
        max_target_forward_overshoot_ = 0.15;
        max_path_curvature_for_planning_ = 4.0;
        quintic_retry_count_ = 5;

        // ============================================================
        // 近距离目标预测：保留当前仿真版本
        // ============================================================
        direct_predict_time_ = 0.30;

        // ============================================================
        // 状态/通信超时：沿用原实验接口思路
        // ============================================================
        pose_timeout_ = 0.30;
        balance_timeout_ = 0.30;

        control_mode_ = ControlMode::QUINTIC_FAR;

        // ============================================================
        // 输入1：二维码相对位姿
        // ============================================================
        relative_pose_sub_ =
            this->create_subscription<geometry_msgs::msg::Pose2D>(
                "/docking/relative_pose",
                10,
                std::bind(
                    &DockingControlNode::relativePoseCallback,
                    this,
                    std::placeholders::_1));

        // ============================================================
        // 输入2：原实验平衡控制接口
        // 使用：current_speed, gyro_z
        // ============================================================
        balance_data_sub_ =
            this->create_subscription<trackmsg::msg::BalanceDate>(
                "/balance_date",
                20,
                std::bind(
                    &DockingControlNode::balanceDataCallback,
                    this,
                    std::placeholders::_1));

        // ============================================================
        // 输出：保留原实验程序接口
        // ============================================================
        target_speed_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/docking/target_speed", 10);

        // 单位：deg/s，直接对应原 balance_controller 的接口。
        target_yaw_rate_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/docking/target_yaw_rate", 10);

        target_curvature_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/docking/target_curvature", 10);

        target_radius_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/docking/target_radius", 10);

        auto_cmd_vel_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/docking/auto_cmd_vel", 10);

        finished_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/docking/finished", 10);

        const int period_ms =
            static_cast<int>(1000.0 / control_frequency_);

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(
                    &DockingControlNode::controlLoop,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "============================================");
        RCLCPP_INFO(
            this->get_logger(),
            "Relative-coordinate hybrid docking controller started");
        RCLCPP_INFO(
            this->get_logger(),
            "Input: /docking/relative_pose + /balance_date");
        RCLCPP_INFO(
            this->get_logger(),
            "FAR: quintic + Stanley | NEAR: direct target + yaw-rate FF");
        RCLCPP_INFO(
            this->get_logger(),
            "direct_enter=%.2f m, quintic_enter=%.2f m, Stanley k=%.2f",
            direct_enter_distance_,
            quintic_enter_distance_,
            stanley_gain_);
        RCLCPP_INFO(
            this->get_logger(),
            "relative pose convention: x=forward, y=left, theta=target yaw relative to follower");
        RCLCPP_INFO(
            this->get_logger(),
            "target yaw-rate feedforward gain=%.2f",
            target_yaw_rate_ff_gain_);
    }

private:
    // ================================================================
    // 工具函数
    // ================================================================
    static double clampValue(
        double value,
        double min_value,
        double max_value)
    {
        return std::max(min_value, std::min(value, max_value));
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    static double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    static double rad2deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }

    const char *modeName() const
    {
        return (control_mode_ == ControlMode::QUINTIC_FAR)
                   ? "QUINTIC_FAR"
                   : "DIRECT_NEAR";
    }

    // ================================================================
    // 相对位姿回调
    // ================================================================
    void relativePoseCallback(
        const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        relative_target_pose_.x = msg->x;
        relative_target_pose_.y = msg->y;
        relative_target_pose_.theta = normalizeAngle(msg->theta);

        last_pose_time_ = this->now().seconds();
        has_relative_pose_ = true;

        if (!has_relative_distance_)
        {
            const double d = std::hypot(msg->x, msg->y);
            last_relative_distance_ = d;
            has_relative_distance_ = true;
        }
    }

    // ================================================================
    // 原实验平衡控制数据接口
    // ================================================================
    void balanceDataCallback(
        const trackmsg::msg::BalanceDate::SharedPtr msg)
    {
        current_speed_ = static_cast<double>(msg->current_speed);
        current_gyro_z_deg_ = static_cast<double>(msg->gyro_z);

        last_balance_time_ = this->now().seconds();
        has_balance_data_ = true;

        if (current_speed_ < 0.0)
            current_speed_ = 0.0;

        current_speed_ = std::min(
            current_speed_,
            max_command_speed_);
    }

    // ================================================================
    // 用相对位姿 + 自身运动估计目标角速度
    // theta_rel = theta_target - theta_follower
    // => omega_target = d(theta_rel)/dt + omega_follower
    // ================================================================
    void updateTargetMotionEstimate(double dt)
    {
        if (!has_relative_pose_ || dt <= 1e-4)
            return;

        const double current_distance =
            std::hypot(
                relative_target_pose_.x,
                relative_target_pose_.y);

        if (has_last_relative_theta_)
        {
            const double dtheta_rel = normalizeAngle(
                relative_target_pose_.theta - last_relative_theta_);

            const double relative_yaw_rate = dtheta_rel / dt;
            const double follower_yaw_rate =
                deg2rad(current_gyro_z_deg_);

            const double raw_target_yaw_rate =
                relative_yaw_rate + follower_yaw_rate;

            filtered_target_yaw_rate_rad_ =
                target_yaw_rate_filter_alpha_ * raw_target_yaw_rate +
                (1.0 - target_yaw_rate_filter_alpha_) *
                    filtered_target_yaw_rate_rad_;

            filtered_target_yaw_rate_rad_ = clampValue(
                filtered_target_yaw_rate_rad_,
                -max_target_yaw_rate_rad_,
                max_target_yaw_rate_rad_);
        }

        last_relative_theta_ = relative_target_pose_.theta;
        has_last_relative_theta_ = true;

        // ------------------------------------------------------------
        // 保留原实验程序的速度估计思想：
        // relative_speed = d(distance)/dt
        // estimated_target_speed = current_speed + relative_speed
        // ------------------------------------------------------------
        if (has_last_relative_distance_)
        {
            double relative_speed =
                (current_distance - last_relative_distance_) / dt;

            relative_speed = clampValue(
                relative_speed,
                -max_relative_speed_,
                max_relative_speed_);

            filtered_relative_speed_ =
                relative_speed_alpha_ * relative_speed +
                (1.0 - relative_speed_alpha_) *
                    filtered_relative_speed_;

            double estimated_target_speed =
                current_speed_ + filtered_relative_speed_;

            estimated_target_speed = clampValue(
                estimated_target_speed,
                0.0,
                max_command_speed_);

            filtered_target_speed_ =
                speed_filter_alpha_ * estimated_target_speed +
                (1.0 - speed_filter_alpha_) * filtered_target_speed_;
        }

        last_relative_distance_ = current_distance;
        has_last_relative_distance_ = true;
    }

    // ================================================================
    // 将相对目标位姿转换到一个局部惯性坐标系
    // 当前车使用局部里程计 local_current_pose_
    // ================================================================
    void updateLocalTargetPose()
    {
        const double c = std::cos(local_current_pose_.theta);
        const double s = std::sin(local_current_pose_.theta);

        local_target_pose_.x =
            local_current_pose_.x +
            c * relative_target_pose_.x -
            s * relative_target_pose_.y;

        local_target_pose_.y =
            local_current_pose_.y +
            s * relative_target_pose_.x +
            c * relative_target_pose_.y;

        local_target_pose_.theta = normalizeAngle(
            local_current_pose_.theta +
            relative_target_pose_.theta);
    }

    // ================================================================
    // 用 balance_date 速度和 gyroZ 积分跟随车的局部位姿
    // 该坐标系只用于短时滚动规划，不要求长期无漂移。
    // ================================================================
    void integrateFollowerLocalPose(double dt)
    {
        if (dt <= 1e-4)
            return;

        const double omega =
            deg2rad(current_gyro_z_deg_);

        const double theta_mid =
            local_current_pose_.theta +
            0.5 * omega * dt;

        local_current_pose_.x +=
            current_speed_ *
            std::cos(theta_mid) *
            dt;

        local_current_pose_.y +=
            current_speed_ *
            std::sin(theta_mid) *
            dt;

        local_current_pose_.theta =
            normalizeAngle(
                local_current_pose_.theta +
                omega * dt);
    }

    // ================================================================
    // 五次多项式
    // p(0)=p0, p(1)=p1
    // p'(0)=v0, p'(1)=v1
    // p''(0)=p''(1)=0
    // ================================================================
    static QuinticResult quintic1D(
        double p0,
        double p1,
        double v0,
        double v1,
        double s)
    {
        const double d = p1 - p0;

        const double a0 = p0;
        const double a1 = v0;
        const double a2 = 0.0;

        const double a3 =
            10.0 * d -
            6.0 * v0 -
            4.0 * v1;

        const double a4 =
            -15.0 * d +
            8.0 * v0 +
            7.0 * v1;

        const double a5 =
            6.0 * d -
            3.0 * v0 -
            3.0 * v1;

        const double s2 = s * s;
        const double s3 = s2 * s;
        const double s4 = s3 * s;
        const double s5 = s4 * s;

        QuinticResult out;

        out.p =
            a0 +
            a1 * s +
            a2 * s2 +
            a3 * s3 +
            a4 * s4 +
            a5 * s5;

        out.dp =
            a1 +
            2.0 * a2 * s +
            3.0 * a3 * s2 +
            4.0 * a4 * s3 +
            5.0 * a5 * s4;

        out.ddp =
            2.0 * a2 +
            6.0 * a3 * s +
            12.0 * a4 * s2 +
            20.0 * a5 * s3;

        return out;
    }

    // ================================================================
    // 用当前局部目标状态生成五次样条
    // ================================================================
    void buildQuinticTrajectory(double tangent_scale)
    {
        trajectory_.clear();

        constexpr std::size_t sample_count = 81;
        trajectory_.reserve(sample_count);

        const double v0x =
            tangent_scale * std::cos(local_current_pose_.theta);
        const double v0y =
            tangent_scale * std::sin(local_current_pose_.theta);

        const double v1x =
            tangent_scale * std::cos(local_target_pose_.theta);
        const double v1y =
            tangent_scale * std::sin(local_target_pose_.theta);

        for (std::size_t i = 0; i < sample_count; ++i)
        {
            const double s =
                static_cast<double>(i) /
                static_cast<double>(sample_count - 1);

            const QuinticResult qx =
                quintic1D(
                    local_current_pose_.x,
                    local_target_pose_.x,
                    v0x,
                    v1x,
                    s);

            const QuinticResult qy =
                quintic1D(
                    local_current_pose_.y,
                    local_target_pose_.y,
                    v0y,
                    v1y,
                    s);

            const double derivative_norm =
                std::hypot(qx.dp, qy.dp);

            PathPoint point;
            point.x = qx.p;
            point.y = qy.p;

            if (derivative_norm > 1e-6)
            {
                point.yaw = std::atan2(qy.dp, qx.dp);

                const double denominator =
                    std::pow(derivative_norm, 3.0);

                point.curvature =
                    (
                        qx.dp * qy.ddp -
                        qy.dp * qx.ddp
                    ) /
                    std::max(denominator, 1e-8);
            }
            else
            {
                point.yaw =
                    (i == 0)
                        ? local_current_pose_.theta
                        : trajectory_.back().yaw;

                point.curvature = 0.0;
            }

            trajectory_.push_back(point);
        }
    }

    // ================================================================
    // 五次样条安全检查
    // ================================================================
    bool quinticTrajectoryIsUnsafe() const
    {
        if (trajectory_.size() < 3)
            return false;

        const double start_to_target =
            std::hypot(
                local_target_pose_.x - local_current_pose_.x,
                local_target_pose_.y - local_current_pose_.y);

        if (start_to_target < 0.05)
            return false;

        // 1. 路径总长度
        double path_length = 0.0;

        for (std::size_t i = 1; i < trajectory_.size(); ++i)
        {
            path_length += std::hypot(
                trajectory_[i].x - trajectory_[i - 1].x,
                trajectory_[i].y - trajectory_[i - 1].y);
        }

        if (path_length >
            max_path_length_ratio_ * start_to_target)
        {
            return true;
        }

        // 2. 是否越过目标车沿自身航向的前方太多
        const double target_tx =
            std::cos(local_target_pose_.theta);
        const double target_ty =
            std::sin(local_target_pose_.theta);

        for (const auto &point : trajectory_)
        {
            const double dx =
                point.x - local_target_pose_.x;
            const double dy =
                point.y - local_target_pose_.y;

            const double forward_projection =
                dx * target_tx +
                dy * target_ty;

            if (forward_projection >
                max_target_forward_overshoot_)
            {
                return true;
            }
        }

        // 3. 最大曲率
        for (const auto &point : trajectory_)
        {
            if (!std::isfinite(point.curvature))
                return true;

            if (std::abs(point.curvature) >
                max_path_curvature_for_planning_)
            {
                return true;
            }
        }

        // 4. 局部方向突然回头
        for (std::size_t i = 1; i < trajectory_.size(); ++i)
        {
            const double dyaw =
                std::abs(
                    normalizeAngle(
                        trajectory_[i].yaw -
                        trajectory_[i - 1].yaw));

            if (dyaw > deg2rad(120.0))
                return true;
        }

        return false;
    }

    // ================================================================
    // 滚动生成远距离轨迹
    // ================================================================
    void generateLocalTrajectory()
    {
        trajectory_.clear();

        const double dx =
            local_target_pose_.x - local_current_pose_.x;
        const double dy =
            local_target_pose_.y - local_current_pose_.y;

        const double distance =
            std::hypot(dx, dy);

        if (distance < 0.05)
        {
            PathPoint p;
            p.x = local_target_pose_.x;
            p.y = local_target_pose_.y;
            p.yaw = local_target_pose_.theta;
            p.curvature = 0.0;
            trajectory_.push_back(p);
            return;
        }

        const double heading_diff =
            std::abs(
                normalizeAngle(
                    local_target_pose_.theta -
                    local_current_pose_.theta));

        double tangent_scale =
            clampValue(
                quintic_tangent_gain_ * distance,
                quintic_tangent_min_,
                quintic_tangent_max_);

        if (heading_diff > 1.0)
            tangent_scale *= 0.70;

        if (heading_diff > 1.6)
            tangent_scale *= 0.45;

        if (heading_diff > 2.2)
            tangent_scale *= 0.25;

        tangent_scale =
            clampValue(
                tangent_scale,
                0.05,
                quintic_tangent_max_);

        int retry_count = 0;

        while (retry_count <= quintic_retry_count_)
        {
            buildQuinticTrajectory(tangent_scale);

            if (!quinticTrajectoryIsUnsafe())
                break;

            tangent_scale *= 0.55;
            tangent_scale = std::max(tangent_scale, 0.03);
            ++retry_count;
        }

        if (quinticTrajectoryIsUnsafe())
        {
            tangent_scale = 0.0;
            buildQuinticTrajectory(tangent_scale);
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Quintic(relative): dist=%.2f headingDiff=%.1fdeg tangent=%.3f retry=%d safe=%s",
            distance,
            rad2deg(heading_diff),
            tangent_scale,
            retry_count,
            quinticTrajectoryIsUnsafe() ? "NO" : "YES");
    }

    // ================================================================
    // 最近点
    // ================================================================
    std::size_t findNearestPoint() const
    {
        if (trajectory_.empty())
            return 0;

        std::size_t nearest_index = 0;
        double min_distance_sq =
            std::numeric_limits<double>::max();

        for (std::size_t i = 0; i < trajectory_.size(); ++i)
        {
            const double dx =
                trajectory_[i].x - local_current_pose_.x;
            const double dy =
                trajectory_[i].y - local_current_pose_.y;

            const double distance_sq =
                dx * dx + dy * dy;

            if (distance_sq < min_distance_sq)
            {
                min_distance_sq = distance_sq;
                nearest_index = i;
            }
        }

        return nearest_index;
    }

    // ================================================================
    // 前视点
    // ================================================================
    std::size_t findTrackingPoint(std::size_t nearest_index) const
    {
        if (trajectory_.empty())
            return 0;

        double accumulated_distance = 0.0;

        for (std::size_t i = nearest_index + 1;
             i < trajectory_.size();
             ++i)
        {
            accumulated_distance += std::hypot(
                trajectory_[i].x - trajectory_[i - 1].x,
                trajectory_[i].y - trajectory_[i - 1].y);

            if (accumulated_distance >= lookahead_distance_)
                return i;
        }

        return trajectory_.size() - 1;
    }

    // ================================================================
    // Stanley
    // ================================================================
    double stanleyControl(
        const PathPoint &point,
        double command_speed,
        double &lateral_error,
        double &heading_error,
        double &path_curvature) const
    {
        const double dx =
            point.x - local_current_pose_.x;
        const double dy =
            point.y - local_current_pose_.y;

        lateral_error =
            -std::sin(point.yaw) * dx +
            std::cos(point.yaw) * dy;

        heading_error =
            normalizeAngle(
                point.yaw -
                local_current_pose_.theta);

        path_curvature = point.curvature;

        const double speed_for_stanley =
            std::abs(current_speed_) + 0.20;

        const double cross_track_term =
            std::atan2(
                stanley_gain_ * lateral_error,
                speed_for_stanley);

        double steering_angle =
            heading_error +
            cross_track_term;

        steering_angle = clampValue(
            steering_angle,
            -1.20,
            1.20);

        const double feedback_curvature =
            std::tan(steering_angle) /
            stanley_virtual_length_;

        double command_curvature =
            path_curvature +
            feedback_curvature;

        command_curvature = clampValue(
            command_curvature,
            -3.0,
            3.0);

        double yaw_rate =
            command_speed * command_curvature;

        yaw_rate = clampValue(
            yaw_rate,
            -max_yaw_rate_rad_,
            max_yaw_rate_rad_);

        return yaw_rate;
    }

    // ================================================================
    // 近距离动态目标跟踪
    // 目标在局部惯性坐标中的位置提前 direct_predict_time_ 秒
    // 再使用 Stanley + 目标角速度前馈
    // ================================================================
    double directTargetControl(
        double command_speed,
        double &lateral_error,
        double &heading_error) const
    {
        const double ref_x =
            local_target_pose_.x +
            filtered_target_speed_ *
                direct_predict_time_ *
                std::cos(local_target_pose_.theta);

        const double ref_y =
            local_target_pose_.y +
            filtered_target_speed_ *
                direct_predict_time_ *
                std::sin(local_target_pose_.theta);

        const double dx =
            ref_x - local_current_pose_.x;
        const double dy =
            ref_y - local_current_pose_.y;

        lateral_error =
            -std::sin(local_target_pose_.theta) * dx +
            std::cos(local_target_pose_.theta) * dy;

        heading_error =
            normalizeAngle(
                local_target_pose_.theta -
                local_current_pose_.theta);

        const double speed_for_stanley =
            std::abs(current_speed_) + 0.20;

        const double cross_track_term =
            std::atan2(
                stanley_gain_ * lateral_error,
                speed_for_stanley);

        double steering_angle =
            heading_error +
            cross_track_term;

        steering_angle = clampValue(
            steering_angle,
            -1.20,
            1.20);

        const double feedback_curvature =
            std::tan(steering_angle) /
            stanley_virtual_length_;

        double feedback_yaw_rate =
            command_speed * feedback_curvature;

        feedback_yaw_rate = clampValue(
            feedback_yaw_rate,
            -max_yaw_rate_rad_,
            max_yaw_rate_rad_);

        double yaw_rate =
            feedback_yaw_rate;

        return clampValue(
            yaw_rate,
            -max_yaw_rate_rad_,
            max_yaw_rate_rad_);
    }

    // ================================================================
    // 速度规划
    // 保留当前仿真的纵向控制逻辑：
    // signed_longitudinal > 0  : 跟随车落后
    // signed_longitudinal < 0  : 跟随车已经超过目标车
    // ================================================================
    double computeCommandSpeed(
        double dt,
        double signed_longitudinal) const
    {
        double target_speed =
            std::max(
                0.0,
                std::min(
                    filtered_target_speed_,
                    max_command_speed_));

        const double distance_error =
            signed_longitudinal;

        double desired_speed;

        if (distance_error >= 0.0)
        {
            desired_speed =
                target_speed +
                distance_gain_ * distance_error;
        }
        else
        {
            desired_speed =
                target_speed +
                catch_down_gain_ * distance_error;
        }

        desired_speed = clampValue(
            desired_speed,
            0.0,
            max_command_speed_);

        if (control_mode_ == ControlMode::DIRECT_TARGET_NEAR)
        {
            desired_speed = std::min(
                desired_speed,
                target_speed + near_max_speed_boost_);
        }

        const double max_delta =
            max_acceleration_ * dt;

        desired_speed = clampValue(
            desired_speed,
            last_command_speed_ - max_delta,
            last_command_speed_ + max_delta);

        desired_speed = clampValue(
            desired_speed,
            0.0,
            max_command_speed_);

        if (control_mode_ == ControlMode::DIRECT_TARGET_NEAR)
        {
            desired_speed = std::min(
                desired_speed,
                target_speed + near_max_speed_boost_);
        }

        return desired_speed;
    }

    // ================================================================
    // 混合控制模式切换
    // ================================================================
    void updateControlMode(double distance)
    {
        const ControlMode old_mode = control_mode_;

        if (control_mode_ == ControlMode::QUINTIC_FAR)
        {
            if (distance <= direct_enter_distance_)
            {
                control_mode_ = ControlMode::DIRECT_TARGET_NEAR;
            }
        }
        else
        {
            if (distance >= quintic_enter_distance_)
            {
                control_mode_ = ControlMode::QUINTIC_FAR;
            }
        }

        if (old_mode != control_mode_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "CONTROL MODE SWITCH: %s -> %s, distance=%.3f m",
                old_mode == ControlMode::QUINTIC_FAR
                    ? "QUINTIC_FAR"
                    : "DIRECT_NEAR",
                control_mode_ == ControlMode::QUINTIC_FAR
                    ? "QUINTIC_FAR"
                    : "DIRECT_NEAR",
                distance);

            trajectory_.clear();
            trajectory_ready_ = false;
        }
    }

    // ================================================================
    // 主控制循环
    // ================================================================
    void controlLoop()
    {
        const double now =
            this->now().seconds();

        // ============================================================
        // 输入有效性
        // ============================================================
        if (!has_relative_pose_)
        {
            publishStop(false);
            return;
        }

        if (!has_balance_data_)
        {
            publishStop(false);
            return;
        }

        if ((now - last_pose_time_) > pose_timeout_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "QR relative pose timeout.");

            publishStop(false);
            return;
        }

        if ((now - last_balance_time_) > balance_timeout_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Balance data timeout.");

            publishStop(false);
            return;
        }

        // ============================================================
        // dt
        // ============================================================
        double dt =
            1.0 / control_frequency_;

        if (last_control_time_ > 0.0)
        {
            dt = now - last_control_time_;
            dt = clampValue(dt, 0.001, 0.10);
        }

        last_control_time_ = now;

        // ============================================================
        // 用真实平衡车速度/gyro 积分短时局部位姿
        // ============================================================
        if (!local_pose_initialized_)
        {
            local_current_pose_ = Pose2{};
            local_pose_initialized_ = true;
        }
        else
        {
            integrateFollowerLocalPose(dt);
        }

        // ============================================================
        // 根据相对二维码姿态恢复目标在局部惯性坐标中的状态
        // ============================================================
        updateLocalTargetPose();

        const double dx =
            local_target_pose_.x -
            local_current_pose_.x;

        const double dy =
            local_target_pose_.y -
            local_current_pose_.y;

        const double distance =
            std::hypot(dx, dy);

        // ============================================================
        // 目标运动状态估计
        // ============================================================
        updateTargetMotionEstimate(dt);

        // ============================================================
        // 纵向误差：沿目标车当前航向
        // ============================================================
        const double signed_longitudinal =
            dx * std::cos(local_target_pose_.theta) +
            dy * std::sin(local_target_pose_.theta);

        // ============================================================
        // 模式切换
        // ============================================================
        updateControlMode(distance);

        // ============================================================
        // 远距离：五次样条
        // ============================================================
        if (control_mode_ == ControlMode::QUINTIC_FAR)
        {
            if (!trajectory_ready_ ||
                (now - last_replan_time_) >= replan_interval_)
            {
                generateLocalTrajectory();
                last_replan_time_ = now;
                trajectory_ready_ = !trajectory_.empty();
            }

            if (!trajectory_ready_ || trajectory_.empty())
            {
                publishStop(false);
                return;
            }
        }

        // ============================================================
        // 速度
        // ============================================================
        const double command_speed =
            computeCommandSpeed(
                dt,
                signed_longitudinal);

        last_command_speed_ = command_speed;

        // ============================================================
        // 横向控制
        // ============================================================
        double lateral_error = 0.0;
        double heading_error = 0.0;
        double path_curvature = 0.0;
        double yaw_rate_rad = 0.0;

        std::size_t nearest_index = 0;
        std::size_t tracking_index = 0;

        if (control_mode_ == ControlMode::QUINTIC_FAR)
        {
            nearest_index = findNearestPoint();
            tracking_index = findTrackingPoint(nearest_index);

            yaw_rate_rad =
                stanleyControl(
                    trajectory_[tracking_index],
                    command_speed,
                    lateral_error,
                    heading_error,
                    path_curvature);
        }
        else
        {
            yaw_rate_rad =
                directTargetControl(
                    command_speed,
                    lateral_error,
                    heading_error);
        }

        // ============================================================
        // 输出转换
        // ============================================================
        const double yaw_rate_deg =
            rad2deg(yaw_rate_rad);

        double curvature = 0.0;

        if (std::abs(command_speed) > 1e-4)
        {
            curvature = yaw_rate_rad / command_speed;
        }

        curvature = clampValue(
            curvature,
            -3.0,
            3.0);

        double radius = 0.0;

        if (std::abs(curvature) > 1e-6)
        {
            radius = 1.0 / curvature;
        }

        // ============================================================
        // 保留原实验接口
        // ============================================================
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = command_speed;
        target_speed_pub_->publish(speed_msg);

        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = yaw_rate_deg;
        target_yaw_rate_pub_->publish(yaw_msg);

        std_msgs::msg::Float64 curvature_msg;
        curvature_msg.data = curvature;
        target_curvature_pub_->publish(curvature_msg);

        std_msgs::msg::Float64 radius_msg;
        radius_msg.data = radius;
        target_radius_pub_->publish(radius_msg);

        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = command_speed;
        cmd_msg.angular.z = yaw_rate_rad;
        auto_cmd_vel_pub_->publish(cmd_msg);

        // 动态跟踪模式不自动“完成”，因此一直发布 false。
        std_msgs::msg::Bool finished_msg;
        finished_msg.data = false;
        finished_pub_->publish(finished_msg);

        // ============================================================
        // 调试日志
        // ============================================================
        static int print_count = 0;
        ++print_count;

        if (print_count >= 25)
        {
            print_count = 0;

            RCLCPP_INFO(
                this->get_logger(),
                "mode=%s dist=%.2f ex=%.2f ey=%.3f eTheta=%.1fdeg "
                "v=%.2f targetV=%.2f targetW=%.2fdeg/s omega=%.2fdeg/s "
                "pathK=%.3f cmdK=%.3f nearX=%.2f nearY=%.2f",
                modeName(),
                distance,
                signed_longitudinal,
                lateral_error,
                rad2deg(heading_error),
                current_speed_,
                filtered_target_speed_,
                rad2deg(filtered_target_yaw_rate_rad_),
                yaw_rate_deg,
                path_curvature,
                curvature,
                relative_target_pose_.x,
                relative_target_pose_.y);
        }
    }

    // ================================================================
    // 停止
    // ================================================================
    void publishStop(bool finished)
    {
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        auto_cmd_vel_pub_->publish(cmd_msg);

        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = 0.0;
        target_speed_pub_->publish(speed_msg);

        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = 0.0;
        target_yaw_rate_pub_->publish(yaw_msg);

        std_msgs::msg::Float64 curvature_msg;
        curvature_msg.data = 0.0;
        target_curvature_pub_->publish(curvature_msg);

        std_msgs::msg::Float64 radius_msg;
        radius_msg.data = 0.0;
        target_radius_pub_->publish(radius_msg);

        std_msgs::msg::Bool finished_msg;
        finished_msg.data = finished;
        finished_pub_->publish(finished_msg);

        last_command_speed_ = 0.0;
    }

private:
    // ================================================================
    // ROS interfaces：保持原实验程序接口
    // ================================================================
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr
        relative_pose_sub_;

    rclcpp::Subscription<trackmsg::msg::BalanceDate>::SharedPtr
        balance_data_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_speed_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_yaw_rate_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_curvature_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_radius_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        auto_cmd_vel_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        finished_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ================================================================
    // 相对位姿 / 局部状态
    // ================================================================
    Pose2 relative_target_pose_;

    // 短时局部惯性坐标系中的跟随车位姿
    Pose2 local_current_pose_;

    // 同一局部惯性坐标系中的目标车位姿
    Pose2 local_target_pose_;

    bool local_pose_initialized_{false};

    // ================================================================
    // balance_date
    // ================================================================
    double current_speed_{0.0};
    double current_gyro_z_deg_{0.0};

    bool has_relative_pose_{false};
    bool has_balance_data_{false};

    double last_pose_time_{0.0};
    double last_balance_time_{0.0};
    double last_control_time_{0.0};

    // ================================================================
    // 目标运动估计
    // ================================================================
    bool has_last_relative_theta_{false};
    double last_relative_theta_{0.0};

    bool has_relative_distance_{false};
    bool has_last_relative_distance_{false};
    double last_relative_distance_{0.0};

    double filtered_relative_speed_{0.0};
    double filtered_target_speed_{0.0};

    double filtered_target_yaw_rate_rad_{0.0};

    // ================================================================
    // 五次样条
    // ================================================================
    std::vector<PathPoint> trajectory_;
    bool trajectory_ready_{false};
    double last_replan_time_{0.0};

    // ================================================================
    // 控制模式
    // ================================================================
    ControlMode control_mode_{ControlMode::QUINTIC_FAR};

    // ================================================================
    // Stanley / 控制参数
    // ================================================================
    double stanley_gain_{1.20};
    double stanley_virtual_length_{0.80};
    double lookahead_distance_{0.50};

    double control_frequency_{50.0};
    double replan_interval_{0.10};

    double max_command_speed_{1.35};
    double max_yaw_rate_rad_{1.20};
    double max_acceleration_{1.00};

    double distance_gain_{0.40};
    double speed_filter_alpha_{0.20};
    double near_max_speed_boost_{0.25};
    double catch_down_gain_{0.80};
    double relative_speed_alpha_{0.20};
    double max_relative_speed_{1.0 / 3.6};
    double direct_enter_distance_{2.50};
    double quintic_enter_distance_{3.00};

    double target_yaw_rate_ff_gain_{0.80};
    double max_target_yaw_rate_rad_{1.20};
    double target_yaw_rate_filter_alpha_{0.30};

    double quintic_tangent_gain_{0.30};
    double quintic_tangent_min_{0.10};
    double quintic_tangent_max_{1.20};
    double max_path_length_ratio_{1.70};
    double max_target_forward_overshoot_{0.15};
    double max_path_curvature_for_planning_{4.0};
    int quintic_retry_count_{5};

    double direct_predict_time_{0.30};

    // ================================================================
    // 超时
    // ================================================================
    double pose_timeout_{0.30};
    double balance_timeout_{0.30};

    // ================================================================
    // 最后输出速度
    // ================================================================
    double last_command_speed_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<DockingControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trackmsg/msg/balance_date.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
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
        stanley_gain_ = 1.20;
        stanley_virtual_length_ = 0.80;
        lookahead_distance_ = 0.50;
        control_frequency_ = 50.0;
        replan_interval_ = 0.10;

        max_command_speed_ = 1.35;
        max_yaw_rate_rad_ = 1.20;
        max_acceleration_ = 1.00;

        distance_gain_ = 0.40;
        speed_filter_alpha_ = 0.20;
        near_max_speed_boost_ = 0.25;
        catch_down_gain_ = 0.80;

        direct_enter_distance_ = 1.20;
        quintic_enter_distance_ = 1.50;

        target_yaw_rate_ff_gain_ = 0.80;
        max_target_yaw_rate_rad_ = 1.20;
        target_yaw_rate_filter_alpha_ = 0.30;

        quintic_tangent_gain_ = 0.30;
        quintic_tangent_min_ = 0.10;
        quintic_tangent_max_ = 1.20;
        max_path_length_ratio_ = 1.70;
        max_target_forward_overshoot_ = 0.15;
        max_path_curvature_for_planning_ = 4.0;
        quintic_retry_count_ = 5;

        direct_predict_time_ = 0.30;

        pose_timeout_ = 0.30;
        balance_timeout_ = 0.30;
        control_mode_ = ControlMode::QUINTIC_FAR;

        aruco_pose_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco/pose", 10,
                std::bind(&DockingControlNode::arucoPoseCallback, this,
                          std::placeholders::_1));

        balance_data_sub_ =
            this->create_subscription<trackmsg::msg::BalanceDate>(
                "/balance_date", 20,
                std::bind(&DockingControlNode::balanceDataCallback, this,
                          std::placeholders::_1));

        // 前车（b2）通过无线发来的真实车速和角速度
        lead_data_sub_ =
            this->create_subscription<trackmsg::msg::BalanceDate>(
                "/balance_date_b2", 20,
                std::bind(&DockingControlNode::leadDataCallback, this,
                          std::placeholders::_1));

        target_speed_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/docking/target_speed", 10);
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

        const int period_ms = static_cast<int>(1000.0 / control_frequency_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&DockingControlNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "ArUco direct docking controller started");
        RCLCPP_INFO(this->get_logger(), "Input: /aruco/pose + /balance_date");
    }

private:
    static double clampValue(double v, double lo, double hi)
    { return std::max(lo, std::min(v, hi)); }

    static double normalizeAngle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static double deg2rad(double d) { return d * M_PI / 180.0; }
    static double rad2deg(double r) { return r * 180.0 / M_PI; }

    const char *modeName() const
    {
        return control_mode_ == ControlMode::QUINTIC_FAR ? "QUINTIC_FAR" : "DIRECT_NEAR";
    }

    void arucoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double aruco_x = msg->pose.position.x;
        double aruco_z = msg->pose.position.z;

        relative_target_pose_.x = aruco_z;
        relative_target_pose_.y = -aruco_x;

        tf2::Quaternion q(
            msg->pose.orientation.x, msg->pose.orientation.y,
            msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        relative_target_pose_.theta = normalizeAngle(-yaw);

        last_pose_time_ = this->now().seconds();
        has_relative_pose_ = true;

        if (!has_relative_distance_)
        {
            last_relative_distance_ = std::hypot(
                relative_target_pose_.x, relative_target_pose_.y);
            has_relative_distance_ = true;
        }
    }

    void balanceDataCallback(const trackmsg::msg::BalanceDate::SharedPtr msg)
    {
        current_speed_ = static_cast<double>(msg->current_speed);
        current_gyro_z_deg_ = static_cast<double>(msg->gyro_z);
        last_balance_time_ = this->now().seconds();
        has_balance_data_ = true;
        if (current_speed_ < 0.0) current_speed_ = 0.0;
        current_speed_ = std::min(current_speed_, max_command_speed_);
    }

    // 前车真实数据回调（通过DDS无线接收）
    void leadDataCallback(const trackmsg::msg::BalanceDate::SharedPtr msg)
    {
        lead_speed_ = static_cast<double>(msg->current_speed);
        lead_gyro_z_deg_ = static_cast<double>(msg->gyro_z);
        has_lead_data_ = true;
    }

    void updateTargetMotionEstimate(double dt)
    {
        if (!has_relative_pose_ || dt <= 1e-4) return;
        const double dist = std::hypot(relative_target_pose_.x, relative_target_pose_.y);

        if (has_last_relative_theta_)
        {
            double dtheta = normalizeAngle(
                relative_target_pose_.theta - last_relative_theta_);
            double rel_yaw_rate = dtheta / dt;
            double follower_yaw = deg2rad(current_gyro_z_deg_);
            // 优先用前车真实角速度，否则用视觉估算
            double raw_target_yaw_rate;
            if (has_lead_data_)
            {
                raw_target_yaw_rate = deg2rad(lead_gyro_z_deg_);
            }
            else
            {
                raw_target_yaw_rate = rel_yaw_rate + follower_yaw;
            }
            filtered_target_yaw_rate_rad_ =
                target_yaw_rate_filter_alpha_ * raw_target_yaw_rate +
                (1.0 - target_yaw_rate_filter_alpha_) * filtered_target_yaw_rate_rad_;
            filtered_target_yaw_rate_rad_ = clampValue(
                filtered_target_yaw_rate_rad_, -max_target_yaw_rate_rad_, max_target_yaw_rate_rad_);
        }
        last_relative_theta_ = relative_target_pose_.theta;
        has_last_relative_theta_ = true;

        // 优先用前车真实车速，否则用距离变化率估算
        if (has_lead_data_)
        {
            filtered_target_speed_ = clampValue(
                lead_speed_, 0.0, max_command_speed_);
        }
        else if (has_last_relative_distance_)
        {
            double rel_speed = (dist - last_relative_distance_) / dt;
            rel_speed = clampValue(rel_speed, -max_relative_speed_, max_relative_speed_);
            filtered_relative_speed_ =
                relative_speed_alpha_ * rel_speed +
                (1.0 - relative_speed_alpha_) * filtered_relative_speed_;
            double est_target = current_speed_ + filtered_relative_speed_;
            est_target = clampValue(est_target, 0.0, max_command_speed_);
            filtered_target_speed_ =
                speed_filter_alpha_ * est_target +
                (1.0 - speed_filter_alpha_) * filtered_target_speed_;
        }
        last_relative_distance_ = dist;
        has_last_relative_distance_ = true;
    }

    void updateLocalTargetPose()
    {
        double c = std::cos(local_current_pose_.theta);
        double s = std::sin(local_current_pose_.theta);
        local_target_pose_.x = local_current_pose_.x +
            c * relative_target_pose_.x - s * relative_target_pose_.y;
        local_target_pose_.y = local_current_pose_.y +
            s * relative_target_pose_.x + c * relative_target_pose_.y;
        local_target_pose_.theta = normalizeAngle(
            local_current_pose_.theta + relative_target_pose_.theta);
    }

    void integrateFollowerLocalPose(double dt)
    {
        if (dt <= 1e-4) return;
        double omega = deg2rad(current_gyro_z_deg_);
        double theta_mid = local_current_pose_.theta + 0.5 * omega * dt;
        local_current_pose_.x += current_speed_ * std::cos(theta_mid) * dt;
        local_current_pose_.y += current_speed_ * std::sin(theta_mid) * dt;
        local_current_pose_.theta = normalizeAngle(local_current_pose_.theta + omega * dt);
    }

    static QuinticResult quintic1D(double p0, double p1, double v0, double v1, double s)
    {
        double d = p1 - p0;
        double a0 = p0, a1 = v0, a2 = 0.0;
        double a3 = 10.0*d - 6.0*v0 - 4.0*v1;
        double a4 = -15.0*d + 8.0*v0 + 7.0*v1;
        double a5 = 6.0*d - 3.0*v0 - 3.0*v1;
        double s2=s*s, s3=s2*s, s4=s3*s, s5=s4*s;
        QuinticResult out;
        out.p = a0+a1*s+a2*s2+a3*s3+a4*s4+a5*s5;
        out.dp = a1+2.0*a2*s+3.0*a3*s2+4.0*a4*s3+5.0*a5*s4;
        out.ddp = 2.0*a2+6.0*a3*s+12.0*a4*s2+20.0*a5*s3;
        return out;
    }

    void buildQuinticTrajectory(double tangent_scale)
    {
        trajectory_.clear();
        constexpr size_t N = 81;
        trajectory_.reserve(N);
        double v0x = tangent_scale * std::cos(local_current_pose_.theta);
        double v0y = tangent_scale * std::sin(local_current_pose_.theta);
        double v1x = tangent_scale * std::cos(local_target_pose_.theta);
        double v1y = tangent_scale * std::sin(local_target_pose_.theta);
        for (size_t i = 0; i < N; ++i)
        {
            double s = (double)i / (double)(N - 1);
            auto qx = quintic1D(local_current_pose_.x, local_target_pose_.x, v0x, v1x, s);
            auto qy = quintic1D(local_current_pose_.y, local_target_pose_.y, v0y, v1y, s);
            double dn = std::hypot(qx.dp, qy.dp);
            PathPoint pt;
            pt.x = qx.p; pt.y = qy.p;
            if (dn > 1e-6)
            {
                pt.yaw = std::atan2(qy.dp, qx.dp);
                double den = std::pow(dn, 3.0);
                pt.curvature = (qx.dp*qy.ddp - qy.dp*qx.ddp) / std::max(den, 1e-8);
            }
            else
            {
                pt.yaw = (i==0) ? local_current_pose_.theta : trajectory_.back().yaw;
                pt.curvature = 0.0;
            }
            trajectory_.push_back(pt);
        }
    }

    bool quinticTrajectoryIsUnsafe() const
    {
        if (trajectory_.size() < 3) return false;
        double s2t = std::hypot(
            local_target_pose_.x - local_current_pose_.x,
            local_target_pose_.y - local_current_pose_.y);
        if (s2t < 0.05) return false;
        double pl = 0.0;
        for (size_t i = 1; i < trajectory_.size(); ++i)
            pl += std::hypot(trajectory_[i].x-trajectory_[i-1].x,
                             trajectory_[i].y-trajectory_[i-1].y);
        if (pl > max_path_length_ratio_ * s2t) return true;
        double tx = std::cos(local_target_pose_.theta);
        double ty = std::sin(local_target_pose_.theta);
        for (auto &pt : trajectory_)
        {
            double dx = pt.x - local_target_pose_.x;
            double dy = pt.y - local_target_pose_.y;
            if (dx*tx + dy*ty > max_target_forward_overshoot_) return true;
        }
        for (auto &pt : trajectory_)
        {
            if (!std::isfinite(pt.curvature)) return true;
            if (std::abs(pt.curvature) > max_path_curvature_for_planning_) return true;
        }
        for (size_t i = 1; i < trajectory_.size(); ++i)
        {
            double dyaw = std::abs(normalizeAngle(trajectory_[i].yaw - trajectory_[i-1].yaw));
            if (dyaw > deg2rad(120.0)) return true;
        }
        return false;
    }

    void generateLocalTrajectory()
    {
        trajectory_.clear();
        double dx = local_target_pose_.x - local_current_pose_.x;
        double dy = local_target_pose_.y - local_current_pose_.y;
        double dist = std::hypot(dx, dy);
        if (dist < 0.05)
        {
            PathPoint p; p.x = local_target_pose_.x; p.y = local_target_pose_.y;
            p.yaw = local_target_pose_.theta; p.curvature = 0.0;
            trajectory_.push_back(p); return;
        }
        double hd = std::abs(normalizeAngle(local_target_pose_.theta - local_current_pose_.theta));
        double ts = clampValue(quintic_tangent_gain_*dist, quintic_tangent_min_, quintic_tangent_max_);
        if (hd > 1.0) ts *= 0.70;
        if (hd > 1.6) ts *= 0.45;
        if (hd > 2.2) ts *= 0.25;
        ts = clampValue(ts, 0.05, quintic_tangent_max_);
        for (int r = 0; r <= quintic_retry_count_; ++r)
        {
            buildQuinticTrajectory(ts);
            if (!quinticTrajectoryIsUnsafe()) break;
            ts *= 0.55;
            ts = std::max(ts, 0.03);
        }
        if (quinticTrajectoryIsUnsafe()) buildQuinticTrajectory(0.0);
    }

    size_t findNearestPoint() const
    {
        if (trajectory_.empty()) return 0;
        size_t idx = 0; double m = std::numeric_limits<double>::max();
        for (size_t i = 0; i < trajectory_.size(); ++i)
        {
            double dx = trajectory_[i].x - local_current_pose_.x;
            double dy = trajectory_[i].y - local_current_pose_.y;
            double d2 = dx*dx+dy*dy;
            if (d2 < m) { m = d2; idx = i; }
        }
        return idx;
    }

    size_t findTrackingPoint(size_t nearest) const
    {
        if (trajectory_.empty()) return 0;
        double acc = 0.0;
        for (size_t i = nearest+1; i < trajectory_.size(); ++i)
        {
            acc += std::hypot(trajectory_[i].x-trajectory_[i-1].x,
                              trajectory_[i].y-trajectory_[i-1].y);
            if (acc >= lookahead_distance_) return i;
        }
        return trajectory_.size()-1;
    }

    double stanleyControl(const PathPoint &pt, double spd,
                         double &le, double &he, double &curv) const
    {
        double dx = pt.x - local_current_pose_.x;
        double dy = pt.y - local_current_pose_.y;
        le = -std::sin(pt.yaw)*dx + std::cos(pt.yaw)*dy;
        he = normalizeAngle(pt.yaw - local_current_pose_.theta);
        curv = pt.curvature;
        double sfs = std::abs(current_speed_) + 0.20;
        double ctt = std::atan2(stanley_gain_*le, sfs);
        double sa = clampValue(he + ctt, -1.20, 1.20);
        double fc = std::tan(sa) / stanley_virtual_length_;
        double cc = clampValue(curv + fc, -3.0, 3.0);
        return clampValue(spd * cc, -max_yaw_rate_rad_, max_yaw_rate_rad_);
    }

    double directTargetControl(double spd, double &le, double &he) const
    {
        double rx = local_target_pose_.x + filtered_target_speed_*direct_predict_time_*std::cos(local_target_pose_.theta);
        double ry = local_target_pose_.y + filtered_target_speed_*direct_predict_time_*std::sin(local_target_pose_.theta);
        double dx = rx - local_current_pose_.x;
        double dy = ry - local_current_pose_.y;
        le = -std::sin(local_target_pose_.theta)*dx + std::cos(local_target_pose_.theta)*dy;
        he = normalizeAngle(local_target_pose_.theta - local_current_pose_.theta);
        double sfs = std::abs(current_speed_) + 0.20;
        double ctt = std::atan2(stanley_gain_*le, sfs);
        double sa = clampValue(he + ctt, -1.20, 1.20);
        double fc = std::tan(sa) / stanley_virtual_length_;
        double yaw = clampValue(spd * fc, -max_yaw_rate_rad_, max_yaw_rate_rad_);
        if (std::abs(spd) > 0.05)
            yaw += target_yaw_rate_ff_gain_ * filtered_target_yaw_rate_rad_;
        return clampValue(yaw, -max_yaw_rate_rad_, max_yaw_rate_rad_);
    }

    double computeCommandSpeed(double dt, double sl) const
    {
        double ts = std::max(0.0, std::min(filtered_target_speed_, max_command_speed_));
        double ds;
        if (sl >= 0.0) ds = ts + distance_gain_ * sl;
        else ds = ts + catch_down_gain_ * sl;
        ds = clampValue(ds, 0.0, max_command_speed_);
        if (control_mode_ == ControlMode::DIRECT_TARGET_NEAR)
            ds = std::min(ds, ts + near_max_speed_boost_);
        double md = max_acceleration_ * dt;
        ds = clampValue(ds, last_command_speed_-md, last_command_speed_+md);
        return clampValue(ds, 0.0, max_command_speed_);
    }

    void updateControlMode(double dist)
    {
        auto old = control_mode_;
        if (control_mode_ == ControlMode::QUINTIC_FAR)
        { if (dist <= direct_enter_distance_) control_mode_ = ControlMode::DIRECT_TARGET_NEAR; }
        else
        { if (dist >= quintic_enter_distance_) control_mode_ = ControlMode::QUINTIC_FAR; }
        if (old != control_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "MODE: %s -> %s, dist=%.3f",
                old==ControlMode::QUINTIC_FAR?"FAR":"NEAR",
                control_mode_==ControlMode::QUINTIC_FAR?"FAR":"NEAR", dist);
            trajectory_.clear(); trajectory_ready_ = false;
        }
    }

    void controlLoop()
    {
        double now = this->now().seconds();
        if (!has_relative_pose_) { publishStop(false); return; }
        if (!has_balance_data_) { publishStop(false); return; }
        if (now - last_pose_time_ > pose_timeout_) { publishStop(false); return; }
        if (now - last_balance_time_ > balance_timeout_) { publishStop(false); return; }

        double dt = 1.0/control_frequency_;
        if (last_control_time_ > 0.0) dt = now - last_control_time_;
        last_control_time_ = now;

        if (!local_pose_initialized_)
        { local_current_pose_ = Pose2{}; local_pose_initialized_ = true; }
        else integrateFollowerLocalPose(dt);

        updateLocalTargetPose();
        double dx = local_target_pose_.x - local_current_pose_.x;
        double dy = local_target_pose_.y - local_current_pose_.y;
        double dist = std::hypot(dx, dy);
        updateTargetMotionEstimate(dt);
        double sl = dx*std::cos(local_target_pose_.theta) + dy*std::sin(local_target_pose_.theta);
        updateControlMode(dist);

        if (control_mode_ == ControlMode::QUINTIC_FAR)
        {
            if (!trajectory_ready_ || now - last_replan_time_ >= replan_interval_)
            { generateLocalTrajectory(); last_replan_time_ = now; trajectory_ready_ = !trajectory_.empty(); }
            if (!trajectory_ready_ || trajectory_.empty()) { publishStop(false); return; }
        }

        double spd = computeCommandSpeed(dt, sl);
        last_command_speed_ = spd;

        double le=0, he=0, curv=0, yaw=0;
        if (control_mode_ == ControlMode::QUINTIC_FAR)
        {
            size_t ni = findNearestPoint();
            size_t ti = findTrackingPoint(ni);
            yaw = stanleyControl(trajectory_[ti], spd, le, he, curv);
        }
        else
        {
            yaw = directTargetControl(spd, le, he);
        }

        double yaw_deg = rad2deg(yaw);
        double c = (std::abs(spd)>1e-4) ? yaw/spd : 0.0;
        c = clampValue(c, -3.0, 3.0);
        double r = (std::abs(c)>1e-6) ? 1.0/c : 0.0;

        std_msgs::msg::Float64 m;
        m.data = spd; target_speed_pub_->publish(m);
        m.data = yaw_deg; target_yaw_rate_pub_->publish(m);
        m.data = c; target_curvature_pub_->publish(m);
        m.data = r; target_radius_pub_->publish(m);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = spd; cmd.angular.z = yaw;
        auto_cmd_vel_pub_->publish(cmd);

        std_msgs::msg::Bool f; f.data = false; finished_pub_->publish(f);

        static int pc = 0;
        if (++pc >= 25)
        {
            pc = 0;
            RCLCPP_INFO(this->get_logger(),
                "mode=%s dist=%.2f sl=%.2f le=%.3f he=%.1fdeg v=%.2f tv=%.2f w=%.1f relX=%.2f relY=%.2f",
                modeName(), dist, sl, le, rad2deg(he),
                current_speed_, filtered_target_speed_, yaw_deg,
                relative_target_pose_.x, relative_target_pose_.y);
        }
    }

    void publishStop(bool finished)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0; cmd.angular.z = 0;
        auto_cmd_vel_pub_->publish(cmd);
        std_msgs::msg::Float64 z; z.data = 0;
        target_speed_pub_->publish(z);
        target_yaw_rate_pub_->publish(z);
        target_curvature_pub_->publish(z);
        target_radius_pub_->publish(z);
        std_msgs::msg::Bool f; f.data = finished;
        finished_pub_->publish(f);
        last_command_speed_ = 0.0;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
    rclcpp::Subscription<trackmsg::msg::BalanceDate>::SharedPtr balance_data_sub_;
    rclcpp::Subscription<trackmsg::msg::BalanceDate>::SharedPtr lead_data_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_yaw_rate_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_curvature_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_radius_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr auto_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Pose2 relative_target_pose_;
    Pose2 local_current_pose_;
    Pose2 local_target_pose_;
    bool local_pose_initialized_{false};
    double current_speed_{0.0};
    double current_gyro_z_deg_{0.0};
    double lead_speed_{0.0};
    double lead_gyro_z_deg_{0.0};
    bool has_lead_data_{false};
    bool has_relative_pose_{false};
    bool has_balance_data_{false};
    bool has_relative_distance_{false};
    double last_pose_time_{0.0};
    double last_balance_time_{0.0};
    double last_control_time_{0.0};

    bool has_last_relative_theta_{false};
    double last_relative_theta_{0.0};
    bool has_last_relative_distance_{false};
    double last_relative_distance_{0.0};
    double filtered_target_yaw_rate_rad_{0.0};
    double filtered_relative_speed_{0.0};
    double relative_speed_alpha_{0.5};
    double max_relative_speed_{1.0};
    double filtered_target_speed_{0.0};

    ControlMode control_mode_{ControlMode::QUINTIC_FAR};
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
    double direct_enter_distance_{1.20};
    double quintic_enter_distance_{1.50};
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
    double pose_timeout_{0.30};
    double balance_timeout_{0.30};
    std::vector<PathPoint> trajectory_;
    bool trajectory_ready_{false};
    double last_replan_time_{0.0};
    double last_command_speed_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DockingControlNode>());
    rclcpp::shutdown();
    return 0;
}

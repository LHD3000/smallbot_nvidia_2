#include "chasis_cpp/pid_movebase_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}
}  // namespace

LowPassFilter::LowPassFilter(int window_size, double cutoff_freq)
{
  configure(window_size, cutoff_freq);
}

void LowPassFilter::configure(int window_size, double cutoff_freq)
{
  window_size_ = std::max(1, window_size);
  cutoff_freq = std::max(0.0, cutoff_freq);
  alpha_ = 1.0 - std::exp(-2.0 * kPi * cutoff_freq);
  window_.clear();
}

double LowPassFilter::update(double new_value)
{
  const double filtered = window_.empty() ? new_value
                                          : window_.back() + alpha_ * (new_value - window_.back());
  window_.push_back(filtered);
  if (static_cast<int>(window_.size()) > window_size_)
  {
    window_.pop_front();
  }
  return filtered;
}

FeedbackLinearizationController::FeedbackLinearizationController(const rclcpp::NodeOptions &options)
    : rclcpp::Node("pid_movebase_node", options)
{
  loadParameters();

  sub_ = this->create_subscription<trackmsg::msg::AngleDistance>(
      base_sub_topic_, rclcpp::QoS(1),
      std::bind(&FeedbackLinearizationController::measurementCb, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(base_pub_topic_, rclcpp::QoS(1));

  using namespace std::chrono_literals;
  control_timer_ = this->create_wall_timer(10ms, std::bind(&FeedbackLinearizationController::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Controller initialized with stabilized 100Hz control loop");
}

FeedbackLinearizationController::~FeedbackLinearizationController()
{
  if (!cmd_pub_)
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Controller shutting down! Stopping robot...");
  geometry_msgs::msg::Twist stop_cmd;
  for (int i = 0; i < 3; ++i)
  {
    cmd_pub_->publish(stop_cmd);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(this->get_logger(), "Safety stop command sent.");
}

void FeedbackLinearizationController::loadParameters()
{
  d_star_ = this->declare_parameter<double>("target_distance", d_star_);
  base_sub_topic_ = this->declare_parameter<std::string>("base_sub_topic", base_sub_topic_);
  base_pub_topic_ = this->declare_parameter<std::string>("base_pub_topic", base_pub_topic_);

  kp_d_ = this->declare_parameter<double>("kp_distance", kp_d_);
  ki_d_ = this->declare_parameter<double>("ki_distance", ki_d_);
  kd_d_ = this->declare_parameter<double>("kd_distance", kd_d_);
  kp_phi_ = this->declare_parameter<double>("kp_angle", kp_phi_);
  ki_phi_ = this->declare_parameter<double>("ki_angle", ki_phi_);
  kd_phi_ = this->declare_parameter<double>("kd_angle", kd_phi_);

  filter_alpha_ = this->declare_parameter<double>("filter_alpha", filter_alpha_);
  const int linear_window = this->declare_parameter<int>("linear_filter_window", 4);
  const double linear_cutoff = this->declare_parameter<double>("linear_filter_cutoff", 1.0);
  vel_filter_.configure(linear_window, linear_cutoff);
  const int angular_window = this->declare_parameter<int>("angular_filter_window", 8);
  const double angular_cutoff = this->declare_parameter<double>("angular_filter_cutoff", 0.4);
  omega_filter_.configure(angular_window, angular_cutoff);

  dead_zone_deg_ = this->declare_parameter<double>("dead_zone_deg", dead_zone_deg_);
  adaptive_deg_ = this->declare_parameter<double>("adaptive_deg", adaptive_deg_);
  adaptive_dim_ = this->declare_parameter<double>("adaptive_dim", adaptive_dim_);
  smooth_factor_ = this->declare_parameter<double>("smooth_factor", smooth_factor_);
  singular_thresh_ = this->declare_parameter<double>("singular_thresh", singular_thresh_);

  v_max_ = this->declare_parameter<double>("max_linear", v_max_);
  omega_max_ = this->declare_parameter<double>("max_angular", omega_max_);
  max_omega_accel_ = this->declare_parameter<double>("max_angular_accel", max_omega_accel_);

  timeout_threshold_ = this->declare_parameter<double>("timeout_threshold", timeout_threshold_);
}

void FeedbackLinearizationController::resetControllerState()
{
  integral_d_ = 0.0;
  integral_phi_ = 0.0;
  last_d_ = 0.0;
  last_phi_ = 0.0;
  filtered_d_dot_ = 0.0;
  filtered_phi_dot_ = 0.0;
  prev_omega_available_ = false;
}

double FeedbackLinearizationController::smoothSaturate(double value, double limit) const
{
  if (limit == 0.0)
  {
    return 0.0;
  }
  if (smooth_factor_ <= 0.0)
  {
    return clamp(value, -limit, limit);
  }

  const double ratio = std::abs(value) / limit;
  const double scale = 1.0 - std::exp(-ratio / smooth_factor_);
  const double saturated = std::tanh(ratio / smooth_factor_) * limit * scale;
  return std::copysign(saturated, value);
}

double FeedbackLinearizationController::adaptiveGain(double current_error, double base_gain, double adaptive_value) const
{
  const double error_value = std::abs(current_error) * 180.0 / kPi;
  const double adj_value = std::abs(adaptive_value);
  if (adj_value <= std::numeric_limits<double>::epsilon())
  {
    return base_gain;
  }
  const double adaptive_factor = 1.0 - std::exp(-4.0 * error_value / adj_value);
  return base_gain * adaptive_factor;
}

void FeedbackLinearizationController::measurementCb(const trackmsg::msg::AngleDistance::SharedPtr msg)
{
  latest_msg_ = *msg;
  last_sub_msg_time_ = this->now();
  has_latest_msg_ = true;
}

void FeedbackLinearizationController::controlLoop()
{
  if (!has_latest_msg_)
  {
    return;
  }

  const rclcpp::Time now = this->now();
  const double elapsed = (now - last_sub_msg_time_).seconds();
  if (elapsed > timeout_threshold_)
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_WARN(this->get_logger(), "Timeout: %.1fs", elapsed);
    return;
  }

  const trackmsg::msg::AngleDistance msg = latest_msg_;
  double dt = 0.05;
  if (has_last_control_time_)
  {
    dt = (now - last_control_time_).seconds();
    dt = clamp(dt, 0.001, 0.1);
  }

  double phi_deg = msg.angle;
  if (std::abs(phi_deg) < dead_zone_deg_)
  {
    phi_deg = 0.0;
  }
  const double phi = phi_deg * kPi / 180.0;
  const double d = msg.distance - d_star_;

  const double raw_d_dot = (d - last_d_) / dt;
  const double raw_phi_dot = (phi - last_phi_) / dt;
  filtered_d_dot_ = filter_alpha_ * raw_d_dot + (1.0 - filter_alpha_) * filtered_d_dot_;
  filtered_phi_dot_ = filter_alpha_ * raw_phi_dot + (1.0 - filter_alpha_) * filtered_phi_dot_;

  const double kp_d_adj = adaptiveGain(d, kp_d_, adaptive_dim_);
  const double kp_phi_adj = adaptiveGain(phi, kp_phi_, adaptive_deg_);

  const double u1 = -(kp_d_adj * d + ki_d_ * integral_d_ + kd_d_ * filtered_d_dot_);
  double u2 = -(kp_phi_adj * phi + ki_phi_ * integral_phi_ + kd_phi_ * filtered_phi_dot_);

  double denominator = std::cos(phi);
  if (std::abs(denominator) < singular_thresh_)
  {
    denominator = std::copysign(singular_thresh_, denominator == 0.0 ? 1.0 : denominator);
    u2 = 0.0;
    RCLCPP_WARN(this->get_logger(), "Entering singular region!");
  }

  double v = (u1 + (d + d_star_) * u2 * std::tan(phi)) / denominator;
  double omega = u2 + (v / (d + d_star_)) * std::sin(phi);
  if (u2 == 0.0)
  {
    omega = 0.0;
  }

  const double v_filtered = vel_filter_.update(v);
  const double omega_filtered = omega_filter_.update(omega);

  v = smoothSaturate(v_filtered, v_max_);
  omega = smoothSaturate(omega_filtered, omega_max_);

  if (prev_omega_available_)
  {
    const double max_delta = max_omega_accel_ * dt;
    double delta = omega - prev_omega_;
    if (std::abs(delta) > max_delta)
    {
      delta = clamp(delta, -max_delta, max_delta);
      omega = prev_omega_ + delta;
    }
  }
  prev_omega_ = omega;
  prev_omega_available_ = true;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = -omega;

  integral_d_ += d * dt;
  integral_phi_ += phi * dt;
  last_d_ = d;
  last_phi_ = phi;
  last_control_time_ = now;
  has_last_control_time_ = true;

  if (ki_d_ != 0.0)
  {
    const double limit = 2.0 / ki_d_;
    integral_d_ = clamp(integral_d_, -limit, limit);
  }
  else
  {
    integral_d_ = 0.0;
  }

  if (ki_phi_ != 0.0)
  {
    const double limit = (30.0 * kPi / 180.0) / ki_phi_;
    integral_phi_ = clamp(integral_phi_, -limit, limit);
  }
  else
  {
    integral_phi_ = 0.0;
  }

  cmd_pub_->publish(cmd);
  RCLCPP_DEBUG(this->get_logger(), "Control: v=%.2f m/s, omega=%.1f deg, AdaptiveGain: %.1f%%",
               v,
               omega * 180.0 / kPi,
               kp_phi_ != 0.0 ? (kp_phi_adj / kp_phi_) * 100.0 : 0.0);

  if (std::isnan(phi) || std::isinf(d))
  {
    resetControllerState();
    has_latest_msg_ = false;
    RCLCPP_WARN(this->get_logger(), "Controller state reset!");
  }
}

#include "chasis_cpp/angle_calculator_utils.hpp"

#include <cmath>
#include <functional>
#include <iomanip>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

AngleCalculator::AngleCalculator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("angle_calculator_node", options)
{
  sub_topic_ = this->declare_parameter<std::string>("ros_output_topic", "/yolotrack/results");
  pub_topic_ = this->declare_parameter<std::string>("output_results", "/angle_distance");
  camera_fx_ = this->declare_parameter<double>("camera_fx", 266.2);
  camera_cx_ = this->declare_parameter<double>("camera_cx", 320.6);
  tau_ = this->declare_parameter<double>("filter_tau", 0.3);

  validateParams();

  focal_length_px_ = camera_fx_;
  principal_x_ = camera_cx_;

  pub_ = this->create_publisher<trackmsg::msg::AngleDistance>(pub_topic_, 10);
  sub_ = this->create_subscription<trackmsg::msg::TrackingResults>(
      sub_topic_, 10, std::bind(&AngleCalculator::callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Angle calculator initialized with tau=%.3f s", tau_);
}

void AngleCalculator::validateParams()
{
  if (tau_ <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid tau %.3f, reset to 0.1", tau_);
    tau_ = 0.1;
  }

  if (camera_fx_ <= 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid camera_fx %.2f, reset to 266.2", camera_fx_);
    camera_fx_ = 266.2;
  }

  if (camera_cx_ < 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid camera_cx %.2f, reset to 320.6", camera_cx_);
    camera_cx_ = 320.6;
  }
}

double AngleCalculator::dynamicAlpha(const rclcpp::Time &current_time)
{
  if (!has_last_time_)
  {
    return 1.0;
  }

  const double dt = (current_time - last_time_).seconds();
  if (dt <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Invalid time delta %.6f s, resetting filter", dt);
    return 1.0;
  }

  return 1.0 - std::exp(-dt / tau_);
}

double AngleCalculator::applyLowpass(double current_angle, const rclcpp::Time &current_time)
{
  const double alpha = dynamicAlpha(current_time);

  double filtered_angle = current_angle;
  if (has_filtered_angle_)
  {
    filtered_angle = alpha * current_angle + (1.0 - alpha) * last_filtered_angle_;
  }

  last_filtered_angle_ = filtered_angle;
  has_filtered_angle_ = true;
  last_time_ = current_time;
  has_last_time_ = true;

  return filtered_angle;
}

void AngleCalculator::callback(const trackmsg::msg::TrackingResults::SharedPtr msg)
{
  const rclcpp::Time now = this->now();
  const double dt = has_last_time_ ? (now - last_time_).seconds() : 0.0;

  if (msg->boxes.empty())
  {
    return;
  }

  const auto &box = msg->boxes.front();
  trackmsg::msg::AngleDistance out_msg;
  out_msg.track_id = box.track_id;
  out_msg.distance = box.distance;
  out_msg.header.stamp = now;
  out_msg.header.frame_id = box.header.frame_id;

  if (focal_length_px_ <= 0.0)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Invalid focal_length_px_ %.3f, skip angle publish", focal_length_px_);
    return;
  }

  const double horizontal_offset = box.x_center - principal_x_;
  const double angle_rad = std::atan(horizontal_offset / focal_length_px_);
  const double raw_angle = angle_rad * 180.0 / kPi;

  const double filtered_angle = applyLowpass(raw_angle, now);
  out_msg.angle = filtered_angle;

  pub_->publish(out_msg);

  RCLCPP_DEBUG(this->get_logger(),
               "ID:%d dt:%.3f Raw:%.2fdeg Filtered:%.2fdeg",
               box.track_id, dt, raw_angle, filtered_angle);
}

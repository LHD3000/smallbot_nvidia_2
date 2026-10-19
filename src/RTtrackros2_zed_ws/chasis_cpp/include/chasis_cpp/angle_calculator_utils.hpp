#ifndef CHASIS_CPP_ANGLE_CALCULATOR_UTILS_HPP
#define CHASIS_CPP_ANGLE_CALCULATOR_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <trackmsg/msg/angle_distance.hpp>
#include <trackmsg/msg/bounding_box.hpp>
#include <trackmsg/msg/tracking_results.hpp>

#include <string>

class AngleCalculator : public rclcpp::Node
{
public:
  explicit AngleCalculator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void validateParams();
  double dynamicAlpha(const rclcpp::Time &current_time);
  double applyLowpass(double current_angle, const rclcpp::Time &current_time);
  void callback(const trackmsg::msg::TrackingResults::SharedPtr msg);

  rclcpp::Publisher<trackmsg::msg::AngleDistance>::SharedPtr pub_;
  rclcpp::Subscription<trackmsg::msg::TrackingResults>::SharedPtr sub_;

  std::string sub_topic_;
  std::string pub_topic_;
  double camera_fx_{266.2};
  double camera_cx_{320.6};
  double tau_{0.3};

  double focal_length_px_{0.0};
  double principal_x_{0.0};
  double last_filtered_angle_{0.0};
  bool has_filtered_angle_{false};

  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool has_last_time_{false};
};

#endif // CHASIS_CPP_ANGLE_CALCULATOR_UTILS_HPP

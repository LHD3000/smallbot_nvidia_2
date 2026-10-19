#ifndef CHASIS_CPP_PID_MOVEBASE_UTILS_HPP
#define CHASIS_CPP_PID_MOVEBASE_UTILS_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trackmsg/msg/angle_distance.hpp>

#include <deque>
#include <string>

class LowPassFilter
{
public:
  LowPassFilter(int window_size = 3, double cutoff_freq = 1.2);
  void configure(int window_size, double cutoff_freq);
  double update(double new_value);

private:
  int window_size_{3};
  double alpha_{1.0};
  std::deque<double> window_;
};

class FeedbackLinearizationController : public rclcpp::Node
{
public:
  explicit FeedbackLinearizationController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~FeedbackLinearizationController() override;

private:
  void loadParameters();
  void resetControllerState();
  double smoothSaturate(double value, double limit) const;
  double adaptiveGain(double current_error, double base_gain, double adaptive_value) const;
  void measurementCb(const trackmsg::msg::AngleDistance::SharedPtr msg);
  void controlLoop();

  rclcpp::Subscription<trackmsg::msg::AngleDistance>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  LowPassFilter vel_filter_;
  LowPassFilter omega_filter_;

  trackmsg::msg::AngleDistance latest_msg_;
  rclcpp::Time last_sub_msg_time_{0, 0, RCL_ROS_TIME};
  bool has_latest_msg_{false};

  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool has_last_control_time_{false};

  double d_star_{1.2};
  std::string base_sub_topic_{"/angle_distance"};
  std::string base_pub_topic_{"/main/cmd_vel"};

  double kp_d_{1.11};
  double ki_d_{0.0};
  double kd_d_{0.04};
  double kp_phi_{1.08};
  double ki_phi_{0.0};
  double kd_phi_{0.17};

  double filter_alpha_{0.35};
  double dead_zone_deg_{0.3};
  double adaptive_deg_{5.0};
  double adaptive_dim_{0.1};
  double smooth_factor_{0.3};
  double singular_thresh_{0.15};

  double v_max_{0.7};
  double omega_max_{1.2};
  double max_omega_accel_{1.5};

  double integral_d_{0.0};
  double integral_phi_{0.0};
  double last_d_{0.0};
  double last_phi_{0.0};
  double filtered_d_dot_{0.0};
  double filtered_phi_dot_{0.0};

  double prev_omega_{0.0};
  bool prev_omega_available_{false};

  double timeout_threshold_{0.5};
};

#endif // CHASIS_CPP_PID_MOVEBASE_UTILS_HPP

#include <exception>

#include "chasis_cpp/pid_movebase_utils.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<FeedbackLinearizationController>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("pid_movebase_node"), "Unhandled exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#include <exception>

#include "chasis_cpp/angle_calculator_utils.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<AngleCalculator>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("angle_calculator_node"), "Unhandled exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

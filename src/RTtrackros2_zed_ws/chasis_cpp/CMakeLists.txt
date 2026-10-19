cmake_minimum_required(VERSION 3.8)
project(chasis_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trackmsg REQUIRED)

include_directories(
  include
)

add_executable(angle_calculator_node
  src/angle_calculator_utils.cpp
  src/angle_calculator_node.cpp
)

ament_target_dependencies(angle_calculator_node
  rclcpp
  trackmsg
)

add_executable(pid_movebase_node
  src/pid_movebase_utils.cpp
  src/pid_movebase_node.cpp
)

ament_target_dependencies(pid_movebase_node
  rclcpp
  geometry_msgs
  trackmsg
)

install(TARGETS angle_calculator_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pid_movebase_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

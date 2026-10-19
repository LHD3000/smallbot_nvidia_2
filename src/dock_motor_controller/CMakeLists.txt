cmake_minimum_required(VERSION 3.8)
project(dock_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)
add_executable(dock_motor_controller
  src/dock_motor_controller.cpp
)
target_include_directories(dock_motor_controller PRIVATE
  ${GPIOD_INCLUDE_DIRS}
)

target_link_directories(dock_motor_controller PRIVATE
  ${GPIOD_LIBRARY_DIRS}
)

target_link_libraries(dock_motor_controller
  ${GPIOD_LIBRARIES}
)
ament_target_dependencies(
  dock_motor_controller
  rclcpp
  std_msgs
)

target_compile_features(dock_motor_controller PUBLIC cxx_std_17)

install(
  TARGETS
  dock_motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
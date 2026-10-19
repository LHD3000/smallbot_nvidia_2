cmake_minimum_required(VERSION 3.8)
project(balance_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trackmsg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_subdirectory(asio-driver)
add_subdirectory(control-algorithm)

add_executable(balance_track_node
  src/node.cpp
  src/utils.cpp
)

target_include_directories(balance_track_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(balance_track_node
  asio_driver
  asio_driver_special
  control_algorithm
)

ament_target_dependencies(balance_track_node
  rclcpp
  geometry_msgs
  trackmsg
  sensor_msgs
  tf2
)
add_executable(docking_control_node
    src/docking_control_node.cpp
)

ament_target_dependencies(
    docking_control_node
    rclcpp
    geometry_msgs
    std_msgs
    sensor_msgs
    trackmsg
)

install(
    TARGETS
        docking_control_node
    DESTINATION
        lib/${PROJECT_NAME}
)
# ==========================================
# add_executable(balance_track_node_v2
#   src/node_v2.cpp
#   src/utils.cpp
# )

# target_include_directories(balance_track_node_v2 PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_link_libraries(balance_track_node_v2
#   asio_driver
#   asio_driver_special
#   control_algorithm
# )

# ament_target_dependencies(balance_track_node_v2
#   rclcpp
#   geometry_msgs
#   trackmsg
#   sensor_msgs
#   tf2
# )
# ==========================================

install(TARGETS balance_track_node balance_track_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

cmake_minimum_required(VERSION 3.10)
project(asio-driver)

option(BUILD_TEST_EXECUTABLE "Build test executable" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(asio_driver SHARED
    src/framework/asio_context.cpp
    src/framework/rate_controller.cpp
    
    src/io/timer.cpp
    src/io/serial.cpp
    src/io/gpio.cpp
    src/io/can.cpp
    src/io/pwm.cpp

    src/utils/bit_op.cpp
    src/utils/bit_coder.cpp
    src/utils/range.cpp
    
    src/component/external_counter.cpp
    
    src/bus/can_bus.cpp

    src/device/rc_sbus.cpp
    src/device/buzzer_src.cpp
    src/device/joystick.cpp
)
target_include_directories(
    asio_driver PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_library(asio_driver_special SHARED
    src/special/motor_brt_can.cpp
    src/special/imu_wit_cmp10a.cpp
    src/special/motor_dc_pwm.cpp
    src/special/motor_dc_io.cpp
)
target_include_directories(
    asio_driver_special PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
message(${CMAKE_CURRENT_SOURCE_DIR}/include)
install(
    DIRECTORY include/
    DESTINATION include)
    
install(
    TARGETS asio_driver
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib)
install(
    TARGETS asio_driver_special
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib)
install(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION lib/cmake/${PROJECT_NAME})
include(CMakePackageConfigHelpers) 
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    PATH_VARS PROJECT_NAME)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "0.1.0"
    COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
  )
    

if(BUILD_TEST_EXECUTABLE)
    enable_testing()
    add_subdirectory(test)
endif()

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VERSION "0.1.0")
set(CPACK_PACKAGE_VENDOR "CZQ")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "asio driver, to control hardware devices")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "czq 1448973286@qq.com")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr/local")

include(CPack)


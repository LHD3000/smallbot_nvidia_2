find_package(GTest REQUIRED)
find_package(spdlog REQUIRED)

function (add_test_executable name src)
    add_executable(${name} ${src})
    target_include_directories(${name} PUBLIC include)
    target_link_libraries(${name} PUBLIC asio_driver asio_driver_special)
    install(TARGETS ${name} DESTINATION bin)
endfunction()

function (add_Gtest_executable name src)
    add_executable(${name} ${src})
    target_include_directories(${name} PUBLIC include)
    target_link_libraries(${name} PUBLIC asio_driver asio_driver_special GTest::GTest GTest::Main)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_Gtest_executable(bit_op utils/bit_op.cpp)
add_Gtest_executable(bit_coder utils/bit_coder.cpp)

add_test_executable(serial_send_receive io/serial_send_rec.cpp)
add_test_executable(serial_loop io/serial_loop.cpp)
add_test_executable(serial_rec_hex io/serial_rec_hex.cpp)
add_test_executable(timer_loop io/timer_loop.cpp)
add_test_executable(timer_once io/timer_once.cpp)
add_test_executable(can_loop io/can_loop.cpp)
add_test_executable(can_send_rec io/can_send_rec.cpp)
add_test_executable(pwm io/pwm.cpp)

add_test_executable(signals others/signals.cpp)

add_test_executable(external_counter component/external_counter.cpp)

add_test_executable(rc_sbus device/rc_sbus.cpp)
add_test_executable(buzzer_src device/buzzer_src.cpp)
add_test_executable(joystick device/joystick.cpp)

add_test_executable(imu_wit_cmp10a special/imu_wit_cmp10a.cpp)
add_test_executable(motor_brt_can_mono special/motor_brt_can_mono.cpp)
target_link_libraries(motor_brt_can_mono PUBLIC spdlog::spdlog)
add_test_executable(motor_brt_can_multi special/motor_brt_can_multi.cpp)
add_test_executable(motor_dc_pwm special/motor_dc_pwm.cpp)
add_test_executable(motor_dc_io special/motor_dc_io.cpp)


add_test_executable(can_block io/can_block.cpp)
target_link_libraries(can_block PUBLIC spdlog::spdlog)
add_test_executable(can_epoll io/can_epoll.cpp)
target_link_libraries(can_epoll PUBLIC spdlog::spdlog)

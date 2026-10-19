#include "asio_driver_special/motor_dc_io.hpp"
#include "asio_driver/framework/rate_controller.hpp"
#include <iostream>
#include <thread>

#include "../test_config.hpp"

int main()
{
    std::cout << "TEST:Motor DC PWM" << std::endl;
    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    AD::Special::MotorDC_Mode1 motor(context);
    AD::Special::MotorDC_Mode1::Config cfg1{
        .pinDirection = 113,
        .pinBrake = 46,
        .pinFeedback = 35,
        .pinPWM = 114,
    };
    AD::Special::MotorDC_Mode1::Config cfg2{.pinDirection = 63, .pinBrake = 62, .pinFeedback = 101, .pinPWM = 47};
    if (!motor.Open(cfg2))
    {
        std::cout << "Motor DC PWM opened failed" << std::endl;
    }

    std::cout << "TEST:Motor DC Direction" << std::endl;
    motor.SetDirection(AD::Special::MotorDC_Mode1::Direction::Clockwise, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    motor.SetDirection(AD::Special::MotorDC_Mode1::Direction::CounterClockwise, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    motor.SetDirection(AD::Special::MotorDC_Mode1::Direction::Stop, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "TEST:Motor DC Brake" << std::endl;
    motor.SetDirection(AD::Special::MotorDC_Mode1::Direction::Clockwise, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    motor.SetBrake(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    motor.SetDirection(AD::Special::MotorDC_Mode1::Direction::Stop, 1);

    std::cout << "TEST:Motor DC Feedback" << std::endl;
    AD::RateController rateController(std::chrono::milliseconds(1000));
    motor.SetDirection(AD::Special::MotorDC_Mode1::Direction::Clockwise, 1);
    motor.SetBrake(false);
    while (context->Running())
    {
        if (rateController.IsArrive())
        {
            int count = motor.GetFeedback();
            std::cout << "loop end" << count << std::endl;
        }
        context->SpinOnce();
    }
    motor.SetDirection(AD::Special::MotorDC_Mode1::Direction::Stop, 1);

    return 0;
}
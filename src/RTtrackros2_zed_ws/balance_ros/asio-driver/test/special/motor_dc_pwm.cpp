#include "asio_driver_special/motor_dc_pwm.hpp"
#include <iostream>
#include <thread>

#include "../test_config.hpp"

int main()
{
    std::cout << "TEST:Motor DC PWM" << std::endl;
    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    AD::Special::MotorDCPwm motor(context);
    AD::Special::MotorDCPwm::Config cfg{
        .pinDirection = LeftMotorDirPin,
        .pinBrake = 101,
        .pinFeedback = 113,
        .pinPWM = LeftMotorPWMPin,
    };
    if(!motor.Open(cfg)){
        std::cout << "Motor DC PWM opened failed" << std::endl;
    }

    while (context->Running())
    {
        motor.SetControl(AD::Special::MotorDCPwm::Direction::Clockwise);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        motor.SetControl(AD::Special::MotorDCPwm::Direction::CounterClockwise);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        motor.SetControl(AD::Special::MotorDCPwm::Direction::Stop);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "loop end" << std::endl;
        context->SpinOnce();
        context->WaitRate(20);
    }
    motor.SetControl(AD::Special::MotorDCPwm::Direction::Stop);
    return 0;
}
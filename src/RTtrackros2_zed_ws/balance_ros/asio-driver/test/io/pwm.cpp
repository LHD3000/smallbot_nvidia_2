#include "asio_driver/io/pwm.hpp"
#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    std::cout << "TEST:PWM" << std::endl;
    AD::PWM pwm;
    if (!pwm.Open(0))
    {
        std::cerr << "Failed to open PWM" << std::endl;
        return -1;
    }
    pwm.SetPeriod(std::chrono::nanoseconds(2000));
    pwm.SetDuty(0.5);
    pwm.SetEnable(true);
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        pwm.SetDuty(0.2);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        pwm.SetDuty(0.8);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        pwm.SetDuty(0.5);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        pwm.SetEnable(false);
    }
    pwm.Close();
    return 0;
}
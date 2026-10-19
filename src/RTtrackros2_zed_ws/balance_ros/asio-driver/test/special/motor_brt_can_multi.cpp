#include "asio_driver_special/motor_brt_can.hpp"
#include <cmath>
#include <iostream>

#include "../test_config.hpp"

int main()
{
    std::cout << "TEST:Motor BRT CAN Multi" << std::endl;
    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    auto can = std::make_unique<AD::Can>(context);
    AD::CanBus::SharePtr bus = AD::CanBus::Create(context);
    AD::Special::MotorBrtCan motor1(context);
    AD::Special::MotorBrtCan motor2(context);

    motor1.Open(bus, AD::Special::MotorBrtCan::MotorConfig{.address = 3, .poleCount = 14});
    motor2.Open(bus, AD::Special::MotorBrtCan::MotorConfig{.address = 4, .poleCount = 14});
    can->Open(CanBusName);
    bus->Open(std::move(can));

    while (context->Running())
    {
        // if (motor1.HasNewData())
        //
        //    std::cout << "\033[2J\033[0;0H";
        //    std::cout << "speed1: " << motor1.ReadState().erpm << std::endl;
        //
        // if (motor2.HasNewData())
        //
        //    std::cout << "\033[2J\033[1;0H";
        //    std::cout << "speed2: " << motor2.ReadState().erpm << std::endl;
        //
        //std::cout << "\033[2J\033[3;0H";
        std::cout << "speed1: " << motor1.ReadState().erpm << std::endl;
        std::cout << "speed2: " << motor2.ReadState().erpm << std::endl;
        float second =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count() /
            1000.0;
        float current = 500 * (1 + std::sin(second / 1));
        //current = 1000;

        motor1.SetCurrent(current);
        motor1.SendHeartBeat();
        motor2.SetCurrent(current);
        motor2.SendHeartBeat();
        // std::cout << "spinning..." << std::endl;
        context->SpinOnce();
        context->WaitRate(150);
    }
    return 0;
}
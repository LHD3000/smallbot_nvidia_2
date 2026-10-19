#include "../test_config.hpp"
#include "asio_driver_special/motor_brt_can.hpp"
#include <iostream>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

int main()
{
    std::cout << "TEST:Motor BRT CAN MONO" << std::endl;

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/file.log", true);
    auto file_logger = std::make_shared<spdlog::logger>("file_logger", file_sink);
    spdlog::register_logger(file_logger);
    spdlog::set_default_logger(file_logger);
    spdlog::flush_every(std::chrono::seconds(1));

    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    auto can = std::make_unique<AD::Can>(context);
    // can->SetLoopBack(false);
    AD::CanBus::SharePtr bus = AD::CanBus::Create(context);
    AD::Special::MotorBrtCan motor(context);

    can->Open("can0");
    bus->Open(std::move(can));
    motor.Open(bus, AD::Special::MotorBrtCan::MotorConfig{.address = 3, .poleCount = 14});
    // motor.Open(bus, AD::Special::MotorBrtCan::MotorConfig{.address = LeftMotorAddress, .poleCount = 14});
    // motor.Open(bus, AD::Special::MotorBrtCan::MotorConfig{.address = RightMotorAddress, .poleCount = 14});

    while (context->Running())
    {
        motor.SendHeartBeat();
        spdlog::info("request");
        motor.RequestRPM();
        motor.RequestCurrent();

        float second =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count() /
            1000.0;
        float current = 1000 * (1 + std::sin(second / 1));
        // current = 1000;
        motor.SetCurrent(current);
        // std::cout << "cin: " << current << std::endl;

        if (motor.HasNewData())
        {
            // std::cout << "\033[2J\033[0;0H";
            std::cout << "speed: " << motor.ReadState().erpm << ", current: " << motor.ReadState().current << std::endl;
            spdlog::info("speed: {}, current: {}", motor.ReadState().erpm, motor.ReadState().current);
        }
        context->SpinOnce();
        // std::cout << "spin once" << std::endl;
        context->WaitRate(200);
    }
    spdlog::drop_all();
    return 0;
}
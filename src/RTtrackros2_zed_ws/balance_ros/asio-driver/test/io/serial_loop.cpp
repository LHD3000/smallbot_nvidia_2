#include "asio_driver/io/serial.hpp"
#include <chrono>
#include <iostream>
#include <span>

int main()
{
    std::cout << "TEST:Serial Loop Back" << std::endl;

    auto context = AD::AsioContext::CreatAsioContext();
    AD::Serial serial(context);
    serial.Open("/dev/ttyUSB0");

    std::string message = "Hello, world!";
    serial.Send(std::span<const unsigned char>(reinterpret_cast<unsigned char *>(message.data()), message.size()));
    serial.OnReceive([&](const std::span<const unsigned char> &data)
                     { std::cout << "Received: " << std::string(data.begin(), data.end()) << std::endl; });
    context->Spin();
    std::cout << "TEST END" << std::endl;
    return 0;
}
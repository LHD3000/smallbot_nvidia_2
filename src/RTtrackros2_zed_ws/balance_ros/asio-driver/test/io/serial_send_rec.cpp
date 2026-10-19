#include "asio_driver/io/serial.hpp"
#include <chrono>
#include <iostream>
#include <span>

// socat -d -d pty,raw,echo=0 pty,raw,echo=0

int main()
{
    std::cout << "TEST:Serial Send and Receive" << std::endl;

    auto context = AD::AsioContext::CreatAsioContext();
    AD::Serial serial1(context);
    serial1.Open("/dev/pts/1");
    AD::Serial serial2(context);
    serial2.Open("/dev/pts/2");

    std::string message = "Hello, world!";
    serial2.OnReceive([&](const std::span<const unsigned char> &data)
                      { std::cout << "Received: " << std::string(data.begin(), data.end()) << std::endl; });
    serial1.Send(std::span<const unsigned char>(reinterpret_cast<unsigned char *>(message.data()), message.size()));
    context->Spin();
    std::cout << "TEST END" << std::endl;
    return 0;
}
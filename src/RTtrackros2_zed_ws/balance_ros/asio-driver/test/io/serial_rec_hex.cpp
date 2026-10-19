#include "asio_driver/io/serial.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <span>

// socat -d -d pty,raw,echo=0 pty,raw,echo=0

int main()
{
    std::cout << "TEST:Serial receive hex" << std::endl;

    auto context = AD::AsioContext::CreatAsioContext();
    AD::Serial serial(context);
    serial.Open("/dev/ttyUSB0", 100'000, AD::Serial::DataLength::Eightbits, AD::Serial::Parity::Even,
                AD::Serial::StopBits::Two);
    int width = 0;

    auto f = [&](const std::span<const unsigned char> data)
    {
        for (auto c : data)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << ' ';
            width++;
            if (width == 25)
            {
                std::cout << std::endl;
                width = 0;
            }
        }
    };
    serial.OnReceive(f);
    context->Spin();
    std::cout << "TEST END" << std::endl;
    return 0;
}
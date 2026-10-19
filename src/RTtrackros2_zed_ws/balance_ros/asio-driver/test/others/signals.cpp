#include "asio_driver/signals.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <thread>

void signal_handler(int sig)
{
    std::cout << "Interrupt signal received!" << sig << std::endl;
}

int main()
{
    std::cout << "Test: signals!" << std::endl;
    AD::SignalRegeist(AD::SignalType::Interrupt, signal_handler);
    while (true)
    {
        std::cout << "Runing" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return 0;
}

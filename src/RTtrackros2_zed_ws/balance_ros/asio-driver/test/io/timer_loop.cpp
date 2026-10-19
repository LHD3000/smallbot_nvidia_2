#include "asio_driver/io/timer.hpp"
#include <iostream>

int main()
{
    std::cout << "TEST:Timer Loop" << std::endl;
    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    AD::Timer timer = AD::Timer(context);
    timer.AsLoopAlarm(std::chrono::milliseconds(1000), []() { std::cout << "On tick!" << std::endl; });
    timer.Start();
    context->Spin();
    std::cout << "TEST END" << std::endl;
    return 0;
}
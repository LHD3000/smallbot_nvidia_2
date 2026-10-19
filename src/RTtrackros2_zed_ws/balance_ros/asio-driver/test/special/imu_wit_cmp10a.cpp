#include "asio_driver_special/imu_wit_cmp10a.hpp"
#include <iostream>

#include "../test_config.hpp"

int main()
{
    std::cout << "TEST:IMU Wit CMP10A" << std::endl;
    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    AD::Special::IMUWitCmp10a imu(context);
    if (!imu.Open(IMUDeviceName, IMUBaudrate))
    {
        std::cerr << "Failed to open device" << std::endl;
        return -1;
    }
    long start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    while (context->Running())
    {
        if (imu.HasNewData())
        {
            std::cout << "\033[2J\033[0;0H";
            std::cout << imu.Read().ToString() << std::endl;
            long current =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout << "interval:" << current - start_time << "ms" << std::endl;
            start_time = current;
        }
        context->SpinOnce();
        context->WaitRate(400);
    }
    return 0;
}
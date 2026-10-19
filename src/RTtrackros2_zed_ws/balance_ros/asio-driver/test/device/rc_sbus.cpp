#include "asio_driver/device/rc_sbus.hpp"
#include <iostream>

#include "../test_config.hpp"

int main()
{
    std::cout << "TEST:rc sbus" << std::endl;

    auto context = AD::AsioContext::CreatAsioContext();
    AD::RC_Sbus rc(context);
    rc.Open(RCSbusDeviceName);

    while (context->Running())
    {
        std::cout << "\033[2J\033[0;0H";
        if (rc.HasNewData())
        {
            auto frame = rc.ReadFrame();
            std::cout << frame.ToString() << std::endl;
            std::cout << (frame.IsFrameLost() ? "Frame Lost" : "Frame OK") << std::endl;
            std::cout << (frame.IsDisconnect() ? "Disconnect" : "Connect") << std::endl;
            std::cout << frame.Ch15() << "  /  " << frame.Ch16() << std::endl;
        }
        context->SpinOnce();
        context->WaitRate(20);
    }

    std::cout << "TEST END" << std::endl;
    return 0;
}
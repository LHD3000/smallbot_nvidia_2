#include "asio_driver/device/joystick.hpp"
#include "asio_driver/device/joystick_layout.hpp"
#include <iostream>

int main()
{
    std::cout << "TEST:JoyStick" << std::endl;

    auto context = AD::AsioContext::CreatAsioContext();
    AD::Joystick js(context);
    if (!js.Open("/dev/input/js0"))
    {
        std::cerr << "Open JoyStick Failed" << std::endl;
    }
    while (context->Running())
    {
        context->SpinOnce(true);
        auto frame = js.ReadFrame();
        auto aixlefthor = frame.GetAix(AD::AsCode(AD::XBoxAixLayout::LeftHor));
        auto aixleftvec = frame.GetAix(AD::AsCode(AD::XBoxAixLayout::LeftVec));
        std::cout << "Aixlefthor: " << aixlefthor << " Aixleftvec: " << aixleftvec << std::endl;
    }

    std::cout << "TEST FINISHED" << std::endl;
}
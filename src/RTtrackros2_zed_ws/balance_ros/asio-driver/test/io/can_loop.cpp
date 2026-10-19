#include "asio_driver/io/can.hpp"
#include <iomanip>
#include <iostream>
int main()
{
    std::cout << "TEST:Can Loop Back" << std::endl;
    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    AD::Can can(context);
    if (!can.Open("can0"))
    {
        std::cerr << "can not open" << std::endl;
        return -1;
    }
    can.SetLoopBack(true);
    can.SetFilter(0, 0);
    can.OnReceive(
        [&](AD::Can::CanID id, std::span<const uint8_t> data)
        {
            std::cout << "rec can frame:\r\n id: " << std::to_string(id) << "  |  len:" << std::to_string(data.size())
                      << "\r\n data: ";
            for (auto byte : data)
            {
                std::cout << std::hex << std::setw(2) << static_cast<int>(byte) << " ";
            }
            std::cout << std::endl;
            std::cout << std::dec;
            context->Quit();
        });
    can.Send(AD::Can::CanID(123), std::vector<uint8_t>{0x11, 0x22, 0x33, 0x44});
    context->Spin();
    return 0;
}
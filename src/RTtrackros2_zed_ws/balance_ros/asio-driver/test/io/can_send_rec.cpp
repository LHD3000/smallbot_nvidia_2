#include "asio_driver/io/can.hpp"
#include <iomanip>
#include <iostream>

int send = 0, rec = 0;
void OnCan2Receive(AD::Can::CanID id, std::span<const uint8_t> data)
{
    if (!(data[0] == 0x11 && data[1] == 0x22 && data[2] == 0x33 && data[3] == 0x44))
    {
        std::cerr << "data error" << std::endl;
    }
    else
    {
        std::cout << "rec can2 frame" + std::to_string(rec++) << std::endl;
    }
    // std::cout << "rec by can2:\r\n id: " << std::to_string(id) << "  |  len:" << std::to_string(data.size())
    //           << "\r\n data: ";
    // for (auto byte : data)
    //{
    //     std::cout << std::hex << std::setw(2) << static_cast<int>(byte) << " ";
    // }
    // std::cout << std::endl;
    // std::cout << std::dec;
}

int main()
{
    std::cout << "TEST:Can Loop Back" << std::endl;
    AD::AsioContext::SharePtr context = AD::AsioContext::CreatAsioContext();
    AD::Can can1(context);
    if (!can1.Open("can0"))
    {
        std::cerr << "can not open" << std::endl;
        return -1;
    }
    can1.SetLoopBack(false);
    can1.SetFilter(0, 0);
    can1.OnReceive(
        [&](AD::Can::CanID id, std::span<const uint8_t> data)
        {
            // std::cout << "rec by can1:\r\n id: " << std::to_string(id) << "  |  len:" << std::to_string(data.size())
            //           << "\r\n data: ";
            // for (auto byte : data)
            //{
            //     std::cout << std::hex << std::setw(2) << static_cast<int>(byte) << " ";
            // }
            // std::cout << std::endl;
            // std::cout << std::dec;
        });

    AD::Can can2(context);
    if (!can2.Open("can1"))
    {
        std::cerr << "can not open" << std::endl;
        return -1;
    }
    can2.SetLoopBack(false);
    // can2.SetFilter(112, 0xffffffff);
    // attention can1 need manully set up
    can2.OnReceive(OnCan2Receive);
    while (context->Running())
    {
        /*
        for (int i = 0; i < 4; i++)
        {
            if (can1.Send(AD::Can::CanID(i + 1), std::vector<uint8_t>{0x11, 0x22, 0x33, 0x44}))
            {
                std::cout << "send can1 frame" + std::to_string(send++) << std::endl;
            }
            else
            {
                std::cerr << "send can1 frame failed" << std::endl;
            }
            can1.Send(AD::Can::CanID(i + 1), std::vector<uint8_t>{0x11, 0x22, 0x33, 0x00});
            can1.Send(AD::Can::CanID(i + 1), std::vector<uint8_t>{0x11, 0x22, 0x33, 0x01});
            can1.Send(AD::Can::CanID(i + 1), std::vector<uint8_t>{0x11, 0x22, 0x33, 0x02});
        }

        //*/
        can1.Send(AD::Can::CanID(1), std::vector<uint8_t>{0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88});
        context->SpinOnce();
        //context->WaitRate(200);
    }
    return 0;
}
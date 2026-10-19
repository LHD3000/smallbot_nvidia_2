#include "asio_driver/framework/asio_context.hpp"
#include "spdlog/spdlog.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/if.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

int main()
{
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/file.log", true);
    auto file_logger = std::make_shared<spdlog::logger>("file_logger", file_sink);
    spdlog::register_logger(file_logger);
    spdlog::set_default_logger(file_logger);
    spdlog::flush_every(std::chrono::seconds(1));

    std::cout << "TEST:Can Block" << std::endl;
    std::string device = "can0";
    int socketfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketfd < 0)
        throw std::runtime_error("Failed to create CAN socket");

    ifreq ifr;
    strcpy(ifr.ifr_name, device.c_str()); // TODO:why can't use std::copy?
    if (ioctl(socketfd, SIOCGIFINDEX, &ifr) == -1)
    {
        close(socketfd);
        throw std::runtime_error("Failed to get interface index");
    }
    sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int ret = bind(socketfd, (const sockaddr *)&addr, sizeof(addr));
    if (ret < 0)
    {
        close(socketfd);
        throw std::runtime_error("Failed to bind CAN socket");
    }

    // Set non-blocking mode
    int flags = fcntl(socketfd, F_GETFL, 0);
    if (flags == -1)
    {
        close(socketfd);
        throw std::runtime_error("Failed to get socket flags");
    }
    if (fcntl(socketfd, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        close(socketfd);
        throw std::runtime_error("Failed to set socket to non-blocking mode");
    }

    AD::AsioContext::SharePtr asio_context = AD::AsioContext::CreatAsioContext();
    can_frame recvBuffer;
    long long ms = 0;
    while (asio_context->Running())
    {
        int ret = 0;
        while ((ret = read(socketfd, &recvBuffer, sizeof(recvBuffer))) > 0)
        {
            if (ret < sizeof(can_frame))
            {
                fprintf(stderr, "Incomplete CAN frame received\n");
                continue;
            }
            if (recvBuffer.can_dlc == 6)
            {
                long long mst = std::chrono::steady_clock::now().time_since_epoch().count();
                std::cout << "rec can frame:\r\n id: " << std::to_string(recvBuffer.can_id)
                          << "  |  len:" << std::to_string(recvBuffer.can_dlc) << "\r\n data: ";
                spdlog::info("rec can frame  id: {}  |  len:{} | data: {}", recvBuffer.can_id, recvBuffer.can_dlc,
                             mst - ms);
                ms = mst;
            }
        }
        asio_context->WaitRate(1000);
    }

    return 0;
}
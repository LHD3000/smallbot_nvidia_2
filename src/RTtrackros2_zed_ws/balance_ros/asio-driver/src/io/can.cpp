#include "asio_driver/io/can.hpp"
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace AD
{

Can::Can(AsioContext::SharePtr context)
{
    this->context = context;
}

Can::~Can()
{
    Close();
}

bool Can::Open(const std::string &device)
{
    this->socketfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
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

    isOpen = context->AddHandler(socketfd, EPOLLET | EPOLLIN | EPOLLOUT,
                                 std::bind(&Can::HandleEvent, this, std::placeholders::_1));
    SetFilter(0, 0);
    SetLoopBack(false);
    return isOpen;
}

bool Can::Close()
{
    if (isOpen && (-1 == close(socketfd) || !context->RemoveHandler(socketfd)))
    {
        return false;
        isOpen = false;
    }
    return true;
}

bool Can::IsOpen() const
{
    return isOpen;
}

void Can::SetFilter(uint32_t id, uint32_t mask)
{
    can_filter filter;
    filter.can_id = id;
    filter.can_mask = mask;
    if (-1 == setsockopt(socketfd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)))
    {
        throw std::runtime_error("Failed to set filter");
    }
}

void Can::SetLoopBack(bool enable)
{
    int loopback = enable ? 1 : 0;
    if (-1 == setsockopt(socketfd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)))
    {
        throw std::runtime_error("Failed to set loopback mode");
    }
    if (-1 == setsockopt(socketfd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback)))
    {
        throw std::runtime_error("Failed to set receive own messages");
    }
}
bool Can::Send(const uint32_t id, std::span<const unsigned char> data)
{
    if (sendQueue.size() <= WarningSendBufferSize)
    {
        can_frame frame{};
        // 根据 ID 自动判断标准帧 / 扩展帧
        if (id > CAN_SFF_MASK)
        {
            frame.can_id = id | CAN_EFF_FLAG;
        }
        else
        {
            frame.can_id = id & CAN_SFF_MASK;
        }

        if (data.size() > CAN_MAX_DLEN)
        {
            std::cerr << "CAN data too long" << std::endl;
            return false;
        }
        frame.can_dlc = data.size();
        std::copy(
            data.begin(),
            data.end(),
            frame.data
        );
        sendQueue.push(frame);
        if (true || sendIdel)
        {
            AsyncSend();
        }
        return true;
    }

    std::cerr
        << "Send buffer size is too large, "
        << "There may be a problem with the CAN bus"
        << std::endl;

    return false;
}


void Can::OnReceive(std::function<void(uint32_t id, std::span<const unsigned char> data)> callback)
{
    this->callback = callback;
}

void Can::AsyncSend()
{
    // std::cout << "AsyncSend" << std::endl;
    sendIdel = false;
    auto sendSize = write(socketfd, &(sendQueue.front()), sizeof(can_frame));
    if (sendSize < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)
            return;
        else
            throw std::runtime_error("Failed to write to CAN socket, with error:" + std::string(strerror(errno)));
    }
    if (sendSize != sizeof(can_frame))
    {
        if (sendSize == 0)
            throw std::runtime_error("Failed to write to CAN socket");
        else if (sendSize > 0)
            throw std::runtime_error("Partial write to CAN socket");
        else
            throw std::runtime_error("Failed to write,with error:" + std::string(strerror(errno)));
    }
    sendQueue.pop();
}

void Can::HandleEvent(const epoll_event &event)
{
    if (event.events & EPOLLIN)
    {
        int ret = 0;
        while ((ret = read(socketfd, &recvBuffer, sizeof(recvBuffer))) > 0)
        {
            if (ret < sizeof(can_frame))
            {
                // fprintf(stderr, "Incomplete CAN frame received\n");
                continue;
            }
            if (callback)
                {
                    // 去掉 SocketCAN 的扩展帧/标准帧标志位
                    const CanID id =recvBuffer.can_id & CAN_EFF_MASK;
                    callback(id,std::span<const unsigned char>(recvBuffer.data,recvBuffer.can_dlc));
                }
        }
        if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            throw std::runtime_error("Failed to read from CAN socket");
        }
    }
    if (event.events & EPOLLOUT)
    {
        // std::cout << "EPOLLOUT" << std::to_string(sendQueue.size()) << std::endl;
        if (sendQueue.empty())
        {
            sendIdel = true;
        }
        else
        {
            // std::cout << "continue AsyncSend" << std::endl;
            AsyncSend();
        }
    }
}

} // namespace AD
#pragma once
#include <functional>
#include <linux/can.h>
#include <queue>
#include <span>
#include <string>

#include "asio_driver/framework/asio_context.hpp"

namespace AD
{
class Can
{
    const static int WarningSendBufferSize = 16;

  public:
    using CanID = uint32_t;

  private:
    std::function<void(CanID id, std::span<const unsigned char> data)> callback;
    AsioContext::SharePtr context;
    std::queue<can_frame> sendQueue;
    can_frame recvBuffer;
    bool isOpen = false;
    bool sendIdel = true;
    int socketfd = -1;

  public:
    Can(AsioContext::SharePtr context);
    ~Can();
    bool Open(const std::string &device);
    bool Close();
    bool IsOpen() const;
    /*
     * Set filter for receiving messages.
     * @param id: CAN ID to filter.
     * @param mask: Mask for filtering.
     * @note:
     *      The mask is a bitwise AND of all bits that must match for the message to be received.
     *          id1&mask==id2&mask,etc.
     * */
    void SetFilter(CanID id, uint32_t mask);
    void SetLoopBack(bool enable);
    bool Send(const CanID id, std::span<const unsigned char> data);
    void OnReceive(std::function<void(CanID id, std::span<const unsigned char> data)> callback);

  private:
    void AsyncSend();
    void HandleEvent(const epoll_event &event);
};
} // namespace AD
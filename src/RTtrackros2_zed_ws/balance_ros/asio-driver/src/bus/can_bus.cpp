#include "asio_driver/bus/can_bus.hpp"

namespace AD
{
CanBus::CanBus(AsioContext::SharePtr context)
{
    this->context = context;
}

CanBus::~CanBus()
{
    if (isOpen)
    {
        Close();
    }
    if (can != nullptr)
    {
        can = nullptr;
        can->Close();
    }
    context = nullptr;
}

CanBus::SharePtr CanBus::Create(AsioContext::SharePtr context)
{
    return std::shared_ptr<CanBus>(new CanBus(context));
}

bool CanBus::Open(std::unique_ptr<Can> can)
{
    if (can == nullptr)
    {
        return false;
    }
    if (isOpen)
    {
        Close();
    }
    this->can = std::move(can);
    this->can->OnReceive(std::bind(&CanBus::OnReceive, this, std::placeholders::_1, std::placeholders::_2));
    isOpen = true;
    return true;
}

bool CanBus::Close()
{
    can.reset();
    isOpen = false;
    return true;
}

bool CanBus::IsOpen() const
{
    return isOpen;
}

bool CanBus::Send(Can::CanID id, std::span<const unsigned char> data)
{
    return can->Send(id, data);
}

bool CanBus::AddListener(Can::CanID id, std::function<void(std::span<const unsigned char>)> callback)
{
    if (callbacks.contains(id))
    {
        throw std::runtime_error("Listener already exists for this ID");
    }
    callbacks[id] = callback;
    return true;
}

bool CanBus::RemoveLinstener(Can::CanID id)
{
    if (!callbacks.contains(id))
    {
        throw std::runtime_error("Listener does not exist for this ID");
    }
    callbacks.erase(id);
    return true;
}

void CanBus::OnReceive(Can::CanID id, std::span<const unsigned char> data)
{
    if (callbacks.contains(id))
    {
        callbacks[id](data);
    }
}

} // namespace AD
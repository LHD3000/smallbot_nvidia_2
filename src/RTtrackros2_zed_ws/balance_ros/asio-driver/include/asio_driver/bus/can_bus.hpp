#pragma once
#include <map>
#include <memory>

#include "asio_driver/framework/asio_context.hpp"
#include "asio_driver/io/can.hpp"

namespace AD
{
class CanBus
{
  private:
    AsioContext::SharePtr context;
    std::unique_ptr<Can> can;
    std::map<Can::CanID, std::function<void(std::span<const unsigned char>)>> callbacks;
    bool isOpen;

  public:
    using SharePtr = std::shared_ptr<CanBus>;
    ~CanBus();
    static SharePtr Create(AsioContext::SharePtr context);
    bool Open(std::unique_ptr<Can> can);
    bool Close();
    bool IsOpen() const;
    bool Send(Can::CanID id, std::span<const unsigned char> data);
    bool AddListener(Can::CanID id, std::function<void(std::span<const unsigned char>)> callback);
    bool RemoveLinstener(Can::CanID id);

  private:
    CanBus(AsioContext::SharePtr context);
    void OnReceive(Can::CanID id, std::span<const unsigned char> data);
};
} // namespace AD

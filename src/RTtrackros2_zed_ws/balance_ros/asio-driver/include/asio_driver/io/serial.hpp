#pragma once
#include <asm/termbits.h>
#include <functional>
#include <span>
#include <string>

#include "asio_driver/framework/asio_context.hpp"

namespace AD
{
class Serial
{
    constexpr static int PreLocatedBufferSize = 1024;

  public:
    enum class Parity
    {
        None,
        Even,
        Odd,
    };

    enum class StopBits
    {
        One,
        Two,
    };

    enum class DataLength
    {
        Fivebits = CS5,
        Sixbits = CS6,
        Sevenbits = CS7,
        Eightbits = CS8,
    };

  private:
    AsioContext::SharePtr context;
    int fd = -1;
    bool isOpen = false;
    bool idelWriteFlag = true;
    std::array<unsigned char, PreLocatedBufferSize> readBuffer;
    std::vector<unsigned char> writeBuffer;
    std::function<void(std::span<const unsigned char> data)> onReadCallback;

  public:
    Serial(AsioContext::SharePtr context);
    ~Serial();
    bool Open(const std::string &device, unsigned int baudrate = 9600, DataLength dataLength = DataLength::Eightbits,
              Parity parity = Parity::None, StopBits stopbits = StopBits::One);
    bool IsOpen();
    bool Close();
    bool Send(std::span<const unsigned char> data);
    // bool Write(const std::string &data);
    void OnReceive(std::function<void(std::span<const unsigned char> data)> onReadCallback);
    // void OnRead(std::function<void(const std::string &data)> onReadCallback);

  private:
    void HandleEvent(const epoll_event &event);
    void AsyncSend();
    void Config(unsigned int baudrate, DataLength dataLength, Parity parity, StopBits stopbits);
};
} // namespace AD

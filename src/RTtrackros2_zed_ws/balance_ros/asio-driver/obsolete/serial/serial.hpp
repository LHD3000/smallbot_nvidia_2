#pragma once
#include <functional>
#include <span>
#include <string>
#include <termios.h>

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

    enum class Baudrate
    {
        Baud9600 = B9600,
        Baud19200 = B19200,
        Baud38400 = B38400,
        Baud115200 = B115200,
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
    bool Open(const std::string &device, Baudrate baudrate = Baudrate::Baud9600,
              DataLength dataLength = DataLength::Eightbits, Parity parity = Parity::None,
              StopBits stopbits = StopBits::One);
    bool IsOpen();
    bool Close();
    bool Send(std::span<const unsigned char> data);
    void OnReceive(std::function<void(std::span<const unsigned char> data)> onReadCallback);

  private:
    void HandleEvent(const epoll_event &event);
    void AsyncSend();

    void SetBasic();
    void SetBaudrate(Baudrate baudrate);
    void SetParity(Parity parity);
    void SetStopBits(StopBits stopbits);
    void SetDataLength(DataLength dataLength);
};
} // namespace AD

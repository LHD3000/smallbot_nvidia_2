#include "asio_driver/io/serial.hpp"
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>

namespace AD
{

Serial::Serial(AsioContext::SharePtr context)
{
    writeBuffer.reserve(PreLocatedBufferSize);
    this->context = context;
    this->isOpen = false;
}
Serial::~Serial()
{
    Close();
}

bool Serial::Open(const std::string &device, unsigned int baudrate, DataLength dataLength, Parity parity,
                  StopBits stopbits)
{
    if (isOpen)
    {
        Close();
    }

    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (-1 == fd)
    {
        throw std::runtime_error("Failed to open serial device with error: " + std::string(strerror(errno)));
    }
    Config(baudrate, dataLength, parity, stopbits);

    isOpen = context->AddHandler(fd, EPOLLIN | EPOLLOUT | EPOLLET,
                                 std::bind(&Serial::HandleEvent, this, std::placeholders::_1));
    return isOpen;
}

bool Serial::Close()
{
    if (isOpen && (-1 == close(fd) || !context->RemoveHandler(fd)))
    {
        isOpen = false;
        return false;
    }
    return true;
}

bool Serial::IsOpen()
{
    return isOpen;
}

bool Serial::Send(std::span<const unsigned char> data)
{
    if (data.size() > PreLocatedBufferSize - writeBuffer.size())
    {
        return false;
    }
    writeBuffer.insert(writeBuffer.end(), data.begin(), data.end());
    if (idelWriteFlag)
    {
        AsyncSend();
    }
    return true;
}
void Serial::OnReceive(std::function<void(std::span<const unsigned char>)> onReadCallback)
{
    this->onReadCallback = onReadCallback;
}

void Serial::HandleEvent(const epoll_event &event)
{
    if (event.events & EPOLLIN)
    {
        // read data from serial device
        ssize_t ret = read(fd, readBuffer.data(), readBuffer.size());
        if (ret > 0)
            onReadCallback(std::span<const unsigned char>(readBuffer.data(), ret));
        // TODO: if buffer is not large enough to hold all data ,some data will be lose
    }
    else if (event.events & EPOLLOUT)
    {
        if (writeBuffer.empty())
        {
            idelWriteFlag = true;
        }
        else
        {
            AsyncSend();
        }
    }
}

void Serial::AsyncSend()
{
    idelWriteFlag = false;
    ssize_t ret = write(fd, writeBuffer.data(), writeBuffer.size());
    if (ret == -1)
    {
        throw std::runtime_error("Failed to write to serial device");
    }
    else if (ret < writeBuffer.size())
    {
        writeBuffer.erase(writeBuffer.begin(), writeBuffer.begin() + ret);
    }
    else
    {
        writeBuffer.clear();
    }
}

void Serial::Config(unsigned int baudrate, DataLength dataLength, Parity parity, StopBits stopbits)
{
    termios2 options;
    if (0 != ioctl(fd, TCGETS2, &options))
    {
        throw std::runtime_error("Failed to get serial device attributes");
    }
    // unknow
    options.c_cflag |= CLOCAL | CREAD;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    // baudrate
    options.c_cflag &= ~CBAUD;
    options.c_cflag |= BOTHER;
    options.c_ispeed = baudrate;
    options.c_ospeed = baudrate;
    // databit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= static_cast<tcflag_t>(dataLength);
    // parity
    if (parity == Parity::None)
    {
        options.c_cflag &= ~PARENB;
    }
    else if (parity == Parity::Even)
    {
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
    }
    else
    {
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
    }
    // stopbits
    if (stopbits == StopBits::One)
        options.c_cflag &= ~CSTOPB;
    else if (stopbits == StopBits::Two)
        options.c_cflag |= CSTOPB;

    if (0 != ioctl(fd, TCSETS2, &options))
    {
        throw std::runtime_error("Failed to set serial device attributes");
    }
}
} // namespace AD

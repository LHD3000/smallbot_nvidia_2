#include "asio_driver/io/serial.hpp"
#include <cstring>
#include <stdexcept>

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

bool Serial::Open(const std::string &device, Baudrate baudrate, DataLength dataLength, Parity parity, StopBits stopbits)
{
    if (isOpen)
    {
        Close();
    }

    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (-1 == fd)
    {
        throw std::runtime_error("Failed to open serial device");
        return false;
    }
    SetBasic();
    SetBaudrate(baudrate);
    SetParity(parity);
    SetStopBits(stopbits);
    SetDataLength(dataLength);

    isOpen = context->AddHandler(fd, EPOLLIN | EPOLLOUT | EPOLLET,
                                 std::bind(&Serial::HandleEvent, this, std::placeholders::_1));
    return isOpen;
}

bool Serial::Close()
{
    if (isOpen && (-1 == close(fd) || !context->RemoveHandler(fd)))
    {
        return false;
        isOpen = false;
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

void Serial::SetBasic()
{
    termios options;
    if (-1 == tcgetattr(fd, &options))
        throw std::runtime_error("Failed to get serial port options: " + std::string(strerror(errno)));
    options.c_cflag |= CLOCAL | CREAD;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    if (-1 == tcsetattr(fd, TCSANOW, &options))
        throw std::runtime_error("Failed to set serial port options: " + std::string(strerror(errno)));
}

void Serial::SetBaudrate(Baudrate baudrate)
{
    termios options;
    if (-1 == tcgetattr(fd, &options))
        throw std::runtime_error("Failed to get serial port options: " + std::string(strerror(errno)));
    cfsetspeed(&options, static_cast<speed_t>(baudrate));
    if (-1 == tcsetattr(fd, TCSANOW, &options))
        throw std::runtime_error("Failed to set serial port options: " + std::string(strerror(errno)));
}

void Serial::SetParity(Parity parity)
{
    termios options;
    if (-1 == tcgetattr(fd, &options))
        throw std::runtime_error("Failed to get serial port options: " + std::string(strerror(errno)));
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
    if (-1 == tcsetattr(fd, TCSANOW, &options))
        throw std::runtime_error("Failed to set serial port options: " + std::string(strerror(errno)));
}

void Serial::SetStopBits(StopBits stopbits)
{
    termios options;
    if (-1 == tcgetattr(fd, &options))
        throw std::runtime_error("Failed to get serial port options: " + std::string(strerror(errno)));
    if (stopbits == StopBits::One)
        options.c_cflag &= ~CSTOPB;
    else if (stopbits == StopBits::Two)
        options.c_cflag |= CSTOPB;
    if (-1 == tcsetattr(fd, TCSANOW, &options))
        throw std::runtime_error("Failed to set serial port options: " + std::string(strerror(errno)));
}

void Serial::SetDataLength(DataLength dataLength)
{
    termios options;
    if (-1 == tcgetattr(fd, &options))
        throw std::runtime_error("Failed to get serial port options: " + std::string(strerror(errno)));
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= static_cast<tcflag_t>(dataLength);
    if (-1 == tcsetattr(fd, TCSANOW, &options))
        throw std::runtime_error("Failed to set serial port options: " + std::string(strerror(errno)));
}
} // namespace AD

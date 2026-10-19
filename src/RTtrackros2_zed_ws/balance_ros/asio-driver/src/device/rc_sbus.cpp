#include "asio_driver/device/rc_sbus.hpp"

namespace AD
{
std::string RC_Sbus::Frame::ToString() const
{
    std::string str;
    str += "Frame:\n";
    str += "  Channels: ";
    for (size_t i = 0; i < 16; i++)
    {
        str += std::to_string(channels[i]) + " ";
    }
    str += "\n  Flag: " + std::to_string(flag) + "\n";
    return str;
}

RC_Sbus::RC_Sbus(AsioContext::SharePtr context)
{
    this->context = context;
    serial = std::make_unique<Serial>(context);
}
RC_Sbus::~RC_Sbus()
{
    Close();
}
bool RC_Sbus::Open(const std::string &device)
{
    if (isOpen)
    {
        Close();
    }
    serial->OnReceive(std::bind(&RC_Sbus::ReadCallback, this, std::placeholders::_1));
    if (serial->Open(device, 100'000, Serial::DataLength::Eightbits, Serial::Parity::Even, Serial::StopBits::Two))
    {
        isOpen = true;
        return true;
    }
    return false;
}
bool RC_Sbus::IsOpen() const
{
    return isOpen;
}
bool RC_Sbus::Close()
{
    return serial->Close();
}
bool RC_Sbus::Write(const Frame &frame)
{
    coder[0] = 0x0f;
    for (size_t i = 0; i < 16; i++)
    {
        coder.SetBits(8 + 11 * i, 18 + 11 * i, frame.channels[i]);
    }
    coder[23] = frame.flag;
    coder[24] = 0x00;
    return serial->Send(std::span<const unsigned char>(coder.data(), Frame::Size));
}

bool RC_Sbus::HasNewData() const
{
    return newData;
}

RC_Sbus::Frame RC_Sbus::ReadFrame()
{
    newData = false;
    return frame;
}

void RC_Sbus::ReadCallback(std::span<const unsigned char> data)
{
    for (char c : data)
    {
        if (recBuf.size() < Frame::Size)
        {
            recBuf.push_back(c);
        }
        else
        {
            if (IsFrameValid())
            {
                DecodeFrame();
                recBuf.clear();
                newData = true;
            }
            else
            {
                recBuf.pop_front();
                recBuf.push_back(c);
            }
        }
    }
}

bool RC_Sbus::IsFrameValid() const
{
    return recBuf[0] == 0x0f && recBuf[24] == 0x00;
}

bool RC_Sbus::DecodeFrame()
{
    std::copy(recBuf.begin(), recBuf.end(), coder.data());
    for (size_t i = 0; i < 16; i++)
    {
        frame.channels[i] = coder.GetBits(8 + 11 * i, 18 + 11 * i);
    }
    frame.flag = recBuf[23];
    return true;
}
} // namespace AD
#pragma once

#include "asio_driver/io/serial.hpp"
#include "asio_driver/utils/bit_coder.hpp"
#include "asio_driver/utils/range.hpp"
#include <deque>

namespace AD
{
class RC_Sbus
{
  public:
    struct Frame
    {
        enum class ModeChannelValue
        {
            Up,
            Centrel,
            Down,
            ErrorNotMode
        };
        constexpr static size_t Size = 25;
        uint16_t channels[16] = {1000};
        uint8_t flag = 0;

        std::string ToString() const;
        bool IsDisconnect() const { return flag & 0x10; }
        bool IsFrameLost() const { return flag & 0x20; }
        bool Ch15() const { return flag & 0x40; }
        bool Ch16() const { return flag & 0x80; }
        float Channel(size_t i, float min = -1, float max = 1) const
        {
            return AD::Utils::Remap(channels[i], 200, 1800, min, max);
        }
        ModeChannelValue ModeChannel(size_t i) const
        {
            switch (channels[i])
            {
            case 200:
                return ModeChannelValue::Up;
            case 1000:
                return ModeChannelValue::Centrel;
            case 1800:
                return ModeChannelValue::Down;
            default:
                return ModeChannelValue::ErrorNotMode;
            }
        }
    };

  private:
    AsioContext::SharePtr context;
    std::unique_ptr<Serial> serial;
    std::deque<unsigned char> recBuf;
    Frame frame;
    bool isOpen = false;
    bool newData = false;
    Utils::BitCoder coder = Utils::BitCoder(25, Utils::BitCoder::ByteOrder::LittleEndian);

  public:
    RC_Sbus(AsioContext::SharePtr context);
    ~RC_Sbus();
    bool Open(const std::string &device);
    bool IsOpen() const;
    bool Close();
    bool Write(const Frame &frame);

    bool HasNewData() const;
    Frame ReadFrame();

  private:
    void ReadCallback(std::span<const unsigned char> data);
    bool IsFrameValid() const;
    bool DecodeFrame();
};
} // namespace AD
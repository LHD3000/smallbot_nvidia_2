#include <asio_driver/utils/bit_coder.hpp>
#include <asio_driver_special/ws2812.hpp>
#include <cassert>
#include <span>

namespace AD::Special
{
int WS2812_Spi::Size()
{
    return colors.size();
}
bool WS2812_Spi::Open(const std::string &dev, int ledCount, bool antiLevel)
{
    this->antiLevel = antiLevel;
    if (!spi.Open(dev))
        return false;
    spi.SetMode(antiLevel ? Spi::MODE2 : Spi::MODE0);
    spi.SetMode(Spi::MODE0);
    spi.SetBitWidth(8);
    spi.SetSpeed(2500000);
    colors.resize(ledCount);
    return true;
}
bool WS2812_Spi::Close()
{
    return spi.Close();
}
bool WS2812_Spi::SetColor(int index, Color color)
{
    assert(index > -1);
    if (colors.size() <= index)
        return false;
    colors[index] = color;
    return true;
}
bool WS2812_Spi::SetColors(std::span<Color> colors)
{
    if (colors.size() != colors.size())
        return false;
    this->colors.assign(colors.begin(), colors.end());
    return true;
}

void Code(unsigned char src, unsigned char *out)
{
    Utils::BitCoder coder(3, Utils::BitCoder::ByteOrder::BigEndian);
    for (int offset = 0; offset < 8; offset++)
    {
        if (src & (1 << offset))
        {
            coder.SetBit(offset * 3 + 0, true);
            coder.SetBit(offset * 3 + 1, true);
            coder.SetBit(offset * 3 + 2, false);
        }
        else
        {
            coder.SetBit(offset * 3 + 0, true);
            coder.SetBit(offset * 3 + 1, false);
            coder.SetBit(offset * 3 + 2, false);
        }
    }
    auto result = coder.AsSpan();
    out[0] = result[0];
    out[1] = result[1];
    out[2] = result[2];
}

bool WS2812_Spi::Update()
{
    int size = colors.size() * 3 * 3;
    unsigned char buffer[size];
    for (int colorIndex = 0; colorIndex < colors.size(); colorIndex++)
    {
        Code(colors[colorIndex].G, buffer + colorIndex * 9 + 0);
        Code(colors[colorIndex].R, buffer + colorIndex * 9 + 3);
        Code(colors[colorIndex].B, buffer + colorIndex * 9 + 6);
    }
    if (antiLevel)
    {
        for (unsigned char &byte : buffer)
        {
            byte = ~byte;
        }
    }
    return spi.Send(std::span<unsigned char>(buffer, buffer + size));
}
} // namespace AD::Special
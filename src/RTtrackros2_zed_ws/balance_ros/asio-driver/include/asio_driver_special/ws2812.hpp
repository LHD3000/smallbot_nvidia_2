#pragma once
#include "asio_driver/io/spi.hpp"
#include <span>
#include <string>
#include <vector>

namespace AD::Special
{
class WS2812_Spi
{
  public:
    struct Color
    {
        unsigned char R;
        unsigned char G;
        unsigned char B;
    };

  private:
    Spi spi;
    std::vector<Color> colors;
    bool antiLevel = false;

  public:
    int Size();
    bool Open(const std::string &dev, int ledCount, bool antiLevel);
    bool Close();
    bool SetColor(int index, Color color);
    bool SetColors(std::span<Color> colors);
    // after invoke this, need delay for some time, usually do not consider this problem.
    bool Update();
};
} // namespace AD::Special
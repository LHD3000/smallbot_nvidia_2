#pragma once
#include "asio_driver/io/gpio.hpp"
#include "asio_driver/io/timer.hpp"
#include <chrono>

namespace AD
{
class Buzzer
{
  public:
    struct Schedual
    {
        std::vector<std::pair<std::chrono::milliseconds, bool>> schedual;
        void Set(int times, std::chrono::milliseconds duration, std::chrono::milliseconds interval);
    };

  private:
    GPIO gpio;
    Timer timer;
    bool activeIsHigh = false;
    bool isOpen = false;
    bool isBeeping = false;
    Schedual schedual;

  public:
    Buzzer(AsioContext::SharePtr context);
    ~Buzzer();
    bool Open(int pin, bool activeIsHigh = true);
    bool Close();
    bool IsOpen() const;
    bool IsBeeping() const;
    void Beep();
    void Beep(Schedual schedual);
    void StopBeep();

  private:
    void OnTimer();
    void Save2Schedule(std::chrono::milliseconds duration, bool isHigh);
};
} // namespace AD
#include "asio_driver/device/buzzer_src.hpp"

namespace AD
{
Buzzer::Buzzer(AsioContext::SharePtr context) : gpio(context), timer(context) {}
Buzzer::~Buzzer()
{
    if (isOpen)
    {
        Close();
    }
}
bool Buzzer::Open(int pin, bool activeIsHigh)
{
    this->activeIsHigh = activeIsHigh;
    isOpen = gpio.Open(pin);
    if (isOpen)
        gpio.SetWorkMode(GPIO::GPIOMode::Output);
    return isOpen;
}
bool Buzzer::Close()
{
    bool success = true;
    if (isBeeping)
        StopBeep();
    if (isOpen)
    {
        timer.Stop();
        success = gpio.Close();
    }
    return success;
}
bool Buzzer::IsOpen() const
{
    return isOpen;
}
bool Buzzer::IsBeeping() const
{
    return isBeeping;
}
void Buzzer::Beep()
{
    gpio.Write(activeIsHigh);
}
void Buzzer::Beep(Schedual schedual)
{
    this->schedual = schedual;
    // TODO: add timer to inmplement schedual
}
void Buzzer::StopBeep()
{
    gpio.Write(!activeIsHigh);
}
} // namespace AD
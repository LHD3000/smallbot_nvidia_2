#pragma once
#include <chrono>
#include <string>
namespace AD
{
class PWM
{
  private:
    int periodFd = -1;
    int dutyFd = -1;
    int enableFd = -1;
    bool isOpen = false;
    bool isEnable = false;
    int period = 0;

  public:
    ~PWM();
    bool Open(int channel, const std::string &device_root = "/sys/class/pwm/pwmchip0");
    bool Close();
    bool isOpened() const;
    bool SetPeriod(std::chrono::nanoseconds period);
    bool SetDuty(float duty);
    bool SetEnable(bool enable);
    bool IsEnabled() const;

  private:
    bool ExportDevice(int channel, const std::string &device_root);
};
} // namespace AD
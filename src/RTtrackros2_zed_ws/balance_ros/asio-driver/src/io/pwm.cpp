#include "asio_driver/io/pwm.hpp"
#include <cassert>
#include <fcntl.h>
#include <stdexcept>
#include <unistd.h>

namespace AD
{
PWM::~PWM()
{
    if (isOpen)
    {
        Close();
    }
}

bool PWM::Open(int channel, const std::string &device_root)
{
    // ensure the device is already exported
    std::string device_path = device_root + "/pwm" + std::to_string(channel);
    if (access(device_path.c_str(), F_OK) == -1)
    {
        if (!ExportDevice(channel, device_root))
        {
            throw std::runtime_error("Failed to export PWM device");
        }
    }
    std::string periodPath = device_path + "/period";
    std::string dutyCyclePath = device_path + "/duty_cycle";
    std::string enablePath = device_path + "/enable";
    int periodFd = open(periodPath.c_str(), O_WRONLY);
    if (periodFd == -1)
    {
        throw std::runtime_error("Failed to open period file");
    }
    int dutyCycleFd = open(dutyCyclePath.c_str(), O_WRONLY);
    if (dutyCycleFd == -1)
    {
        throw std::runtime_error("Failed to open duty cycle file");
    }
    int enableFd = open(enablePath.c_str(), O_WRONLY);
    if (enableFd == -1)
    {
        throw std::runtime_error("Failed to open enable file");
    }
    isOpen = true;
    return true;
}

bool PWM::Close()
{
    if (dutyFd != -1)
        close(dutyFd);
    if (periodFd != -1)
        close(periodFd);
    if (enableFd != -1)
        close(enableFd);
    isOpen = false;
    return true;
}

bool PWM::isOpened() const
{
    return isOpen;
}
bool PWM::SetPeriod(std::chrono::nanoseconds period)
{
    this->period = period.count();
    std::string periodStr = std::to_string(this->period);
    int ret = write(periodFd, periodStr.c_str(), periodStr.size());
    return ret != -1;
}
bool PWM::SetDuty(float duty)
{
    assert(duty >= 0.0 && duty <= 1.0);
    std::string dutyStr = std::to_string(static_cast<int>(period * duty));
    int ret = write(dutyFd, dutyStr.c_str(), dutyStr.size());
    return ret != -1;
}
bool PWM::SetEnable(bool enable)
{
    int ret = write(enableFd, enable ? "1" : "0", 1);
    isEnable = enable;
    return ret != -1;
}
bool PWM::IsEnabled() const
{
    return isEnable;
}

bool PWM::ExportDevice(int channel, const std::string &device_root)
{
    std::string exportPath = device_root + "/export";
    int exportFd = open(exportPath.data(), O_WRONLY);
    if (exportFd == -1)
    {
        throw std::runtime_error("Failed to open export file");
    }
    int ret = write(exportFd, std::to_string(channel).c_str(), std::to_string(channel).size());
    close(exportFd);
    return ret != -1;
}
} // namespace AD
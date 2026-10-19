#pragma once

#include "asio_driver/framework/asio_context.hpp"
#include "asio_driver/io/gpio.hpp"

namespace AD::Special
{
class MotorDCPwm
{
  public:
    enum class Direction
    {
        Clockwise,
        CounterClockwise,
        Stop,
    };
    struct Config
    {
        int pinDirection;
        int pinBrake;
        int pinFeedback;
        int pinPWM;
        // following parameters are not used yet
        int pwmChannel;
        std::string pwmDevice;
        int pwmFrequency;
    };

  private:
    AsioContext::SharePtr context;
    GPIO dirPin;
    GPIO brakePin;
    GPIO feedbackPin;
    GPIO pwmPin;
    unsigned int tick;
    bool isOpen = false;
    bool isBrakeEnabled = false;

  public:
    MotorDCPwm(AsioContext::SharePtr context);
    ~MotorDCPwm();
    bool Open(const Config &config);
    bool Close();
    bool IsOpen() const;
    /// @brief Set the motor ctrl, range from -1.0 to 1.0
    void SetControl(Direction dir);
    void SetBrake(bool enable);
    int GetFeedback() const;

  private:
    void OnFeedback();
};
} // namespace AD::Special
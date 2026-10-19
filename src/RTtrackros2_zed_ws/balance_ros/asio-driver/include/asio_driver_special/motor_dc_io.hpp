#pragma once

#include "asio_driver/component/external_counter.hpp"
#include "asio_driver/framework/asio_context.hpp"
#include "asio_driver/io/gpio.hpp"

namespace AD::Special
{
class MotorDC_Mode1
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
    };

  private:
    AsioContext::SharePtr context;
    GPIO pinDirection;
    GPIO pinBrake;
    GPIO pinPWM;
    ExternalCounter feedbackCounter;
    bool isOpen = false;
    bool isBrakeEnabled = false;

  public:
    MotorDC_Mode1(AD::AsioContext::SharePtr &ctx);
    ~MotorDC_Mode1();
    bool Close();
    bool Open(const Config &cfg);
    bool IsOpen() const;
    void SetDirection(Direction dir, float precent = 1.0f);
    void SetBrake(bool enable);
    int GetFeedback();
};
} // namespace AD::Special
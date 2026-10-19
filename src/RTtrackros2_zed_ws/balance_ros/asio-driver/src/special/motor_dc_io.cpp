#include "asio_driver_special/motor_dc_io.hpp"
#include <thread>

namespace AD::Special
{
MotorDC_Mode1::MotorDC_Mode1(AD::AsioContext::SharePtr &ctx)
    : pinDirection(ctx), pinBrake(ctx), pinPWM(ctx), feedbackCounter(ctx), context(ctx)
{
}

MotorDC_Mode1::~MotorDC_Mode1()
{
    if (isOpen)
    {
        Close();
    }
}

bool MotorDC_Mode1::Close()
{
    SetDirection(Direction::Stop, 0.0f);
    bool ret = true;
    ret &= pinDirection.Close();
    ret &= pinBrake.Close();
    ret &= pinPWM.Close();
    ret &= feedbackCounter.Close();
    isOpen = false;
    return ret;
}

bool MotorDC_Mode1::IsOpen() const
{
    return isOpen;
}

bool MotorDC_Mode1::Open(const Config &cfg)
{
    isOpen = true;
    isOpen &= pinDirection.Open(cfg.pinDirection);
    pinDirection.SetWorkMode(GPIO::GPIOMode::Output);
    isOpen &= pinBrake.Open(cfg.pinBrake);
    pinBrake.SetWorkMode(GPIO::GPIOMode::Output);
    isOpen &= pinPWM.Open(cfg.pinPWM);
    pinPWM.SetWorkMode(GPIO::GPIOMode::Output);
    isOpen &= feedbackCounter.Open(cfg.pinFeedback, GPIO::GPIOEdge::Rising);
    SetDirection(Direction::Stop, 0.0f);
    SetBrake(false);
    return isOpen;
}

void MotorDC_Mode1::SetDirection(Direction dir, float precent)
{
    if (dir == Direction::Clockwise)
    {
        pinDirection.Write(true);
        pinPWM.Write(false);
    }
    else if (dir == Direction::CounterClockwise)
    {
        pinDirection.Write(false);
        pinPWM.Write(false);
    }
    else
    {
        pinPWM.Write(true);
    }
}

void MotorDC_Mode1::SetBrake(bool enable)
{
    isBrakeEnabled = enable;
    pinBrake.Write(!enable);
}

int MotorDC_Mode1::GetFeedback()
{
    return feedbackCounter.Read();
}
} // namespace AD::Special
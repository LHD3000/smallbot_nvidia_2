#include "asio_driver_special/motor_dc_pwm.hpp"
#include <cassert>

namespace AD::Special
{
MotorDCPwm::MotorDCPwm(AsioContext::SharePtr context)
    : context(context), dirPin(context), brakePin(context), feedbackPin(context),pwmPin(context)
{
}

MotorDCPwm::~MotorDCPwm()
{
    if (isOpen)
    {
        Close();
    }
}
bool MotorDCPwm::Open(const Config &config)
{
    isOpen = true;
    isOpen &= dirPin.Open(config.pinDirection);
    //isOpen &= brakePin.Open(config.pinBrake);
    //isOpen &= feedbackPin.Open(config.pinFeedback);
    isOpen &= pwmPin.Open(config.pinPWM);
    //pwmPin.SetPeriod(std::chrono::nanoseconds(1000000000 / config.pwmFrequency));
    //pwmPin.SetDuty(0);
    return isOpen;
}
bool MotorDCPwm::Close()
{
    SetControl(Direction::Stop);
    dirPin.Close();
    brakePin.Close();
    feedbackPin.Close();
    pwmPin.Close();
    return true;
}
bool MotorDCPwm::IsOpen() const
{
    return isOpen;
}
void MotorDCPwm::SetControl(Direction dir)
{
    if(dir == Direction::Clockwise){
        dirPin.Write(true);
        pwmPin.Write(false);
    }
    else if(dir == Direction::CounterClockwise){
        dirPin.Write(false);
        pwmPin.Write(false);
    }
    else{
        pwmPin.Write(true);
    }
    if (isBrakeEnabled)
    {
        brakePin.Write(false);
    }
}
void MotorDCPwm::SetBrake(bool enable)
{
    isBrakeEnabled = enable;
    brakePin.Write(enable);
}
int MotorDCPwm::GetFeedback() const
{
    return tick;
}
} // namespace AD::Special
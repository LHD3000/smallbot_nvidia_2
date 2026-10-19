#pragma once

namespace CtrlAlg
{
template <class T> class PID
{
  protected:
    T kp, ki, kd;

  public:
    PID(T kp, T ki, T kd) : kp(kp), ki(ki), kd(kd){};
    void SetP(T kp) { this->kp = kp; };
    void SetI(T ki) { this->ki = ki; };
    void SetD(T kd) { this->kd = kd; };
    void SetPID(T kp, T ki, T kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    };
    virtual T GetControl(T error, T dt = 1.0) = 0;
};
} // namespace CtrlAlg
#pragma once
#include "ctrl_alg/pid/pid.hpp"
#include <cassert>

namespace CtrlAlg
{
template <class T> class PIDInc : public PID<T>
{
  private:
    T lastSecondError = 0;
    T lastError = 0;

  public:
    PIDInc(T kp, T ki, T kd) : PID<T>(kp, ki, kd) {};
    T GetControl(T error, T dt = 1.0) override
    {
        assert(dt > 0);
        T ctrl = 0.0;
        ctrl += this->kd * (error - 2 * lastError + lastSecondError);
        ctrl += this->kp * (error - lastError);
        ctrl += this->ki * error;
        lastSecondError = lastError;
        lastError = error;
        return ctrl;
    }
};
} // namespace CtrlAlg
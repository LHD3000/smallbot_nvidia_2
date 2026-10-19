#pragma once
#include "ctrl_alg/pid/pid.hpp"
#include <algorithm>
#include <cassert>

namespace CtrlAlg
{
/**
 * PIDPos attention SetMaxIntegralOutputPart to limit the integral part, otherwise I will not work
 */
template <class T> class PIDPos : public PID<T>
{
  private:
    T errorSum = 0;
    T lastError = 0;
    T maxIntegral = 0;

  public:
    PIDPos(T kp, T ki, T kd) : PID<T>(kp, ki, kd){};
    void SetMaxIntegralOutputPart(T maxIntegral)
    {
        assert(maxIntegral >= 0);
        this->maxIntegral = maxIntegral;
    }
    T GetControl(T error, T dt = 1.0) override
    {
        assert(dt > 0);
        errorSum += error * dt;
        //errorSum = std::max(-maxIntegral, std::min(maxIntegral, errorSum));
        
        float derror = (error - lastError) / dt;
        float ctrl = this->kp * error + this->ki * errorSum + this->kd * derror;
        lastError = error;
        return ctrl;
    }
};
} // namespace CtrlAlg
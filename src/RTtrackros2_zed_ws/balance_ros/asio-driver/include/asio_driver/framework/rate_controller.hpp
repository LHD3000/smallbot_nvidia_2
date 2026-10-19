#pragma once

#include <chrono>

namespace AD
{
class RateController
{
  private:
    std::chrono::steady_clock::time_point last;
    std::chrono::milliseconds period;

  public:
    RateController(std::chrono::milliseconds period);
    RateController(int rate);
    bool IsArrive();
    void WaitArrive();
};
} // namespace AD
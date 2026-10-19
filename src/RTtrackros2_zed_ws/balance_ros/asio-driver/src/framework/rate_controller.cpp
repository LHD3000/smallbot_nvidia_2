#include "asio_driver/framework/rate_controller.hpp"
#include <thread>

namespace AD
{
RateController::RateController(std::chrono::milliseconds period) : period(period)
{
    last = std::chrono::steady_clock::now();
}

RateController::RateController(int rate) : RateController(std::chrono::milliseconds(1000 / rate)) {}

bool RateController::IsArrive()
{
    auto now = std::chrono::steady_clock::now();
    if (now - last >= period)
    {
        last = now;
        return true;
    }
    return false;
}
void RateController::WaitArrive()
{
    std::this_thread::sleep_until(last + period);
}
} // namespace AD
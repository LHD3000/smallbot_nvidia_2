#pragma once
#include <optional>
#include <span>

namespace CtrlAlg
{
struct DynamicIndicator
{
    constexpr static double Precent = 0.05;
    std::optional<int> riseTime;
    std::optional<int> peakTime;
    std::optional<double> overshoot;
    std::optional<int> adjustTime;

    void Evaluate(std::span<const double> values, double target);
};

} // namespace CtrlAlg
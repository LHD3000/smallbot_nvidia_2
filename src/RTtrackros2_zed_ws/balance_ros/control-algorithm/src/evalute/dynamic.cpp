#include "ctrl_alg/evaluate/dynamic.hpp"
#include <algorithm>
#include <cassert>

namespace CtrlAlg
{
std::optional<int> EstimatePosition(std::span<const double> values, double value)
{
    assert(values.size() > 0);
    bool lastSign = values[0] < value;
    for (int i = 1; i < values.size(); i++)
    {
        bool sign = values[i] < value;
        if (lastSign == sign)
        {
            continue;
        }
        // return (i - 1 + (value - values[i - 1]) / (values[i] - values[i - 1]));
        return i;
    }
    return std::nullopt;
}

std::optional<int> FindPeek(std::span<const double> values)
{
    assert(values.size() > 2);
    bool lastSign = values[1] < values[0];
    for (int i = 2; i < values.size(); i++)
    {
        bool sign = values[i] < values[i - 1];
        if (sign == lastSign)
        {
            continue;
        }
        return i - 1;
    }
    return std::nullopt;
}

int FindLastInRange(std::span<const double> values, double min, double max)
{
    assert(values.size() > 0);
    for (int i = values.size() - 1; i >= 0; i--)
    {
        if (values[i] <= min && values[i] >= max)
        {
            return i;
        }
    }
    return values.size() - 1;
}

void DynamicIndicator::Evaluate(std::span<const double> values, double target)
{
    double initial = values.front();
    double min = target * (1 - Precent);
    double max = target * (1 + Precent);

    riseTime = EstimatePosition(values, target);
    peakTime = FindPeek(values);
    overshoot = peakTime.has_value() ? std::optional<double>(values[peakTime.value()] - target) : std::nullopt;
    adjustTime = values.size() - FindLastInRange(values, min, max);
}

} // namespace CtrlAlg
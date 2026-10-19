#include "balance_ros/utils.hpp"

#include <limits>

SoftLimter::SoftLimter(double lower_thres, double upper_thres)
    : lower_(lower_thres), upper_(upper_thres)
{
    validateParameters();
}

void SoftLimter::validateParameters() const
{
    assert(lower_ >= 0 && "Lower threshold must be non-negative");
    assert(upper_ > lower_ && "Upper threshold must > lower");
}

double SoftLimter::clamp(double input) const
{
    const double abs_input = std::abs(input);
    const double sign = (input > 0) ? 1.0 : -1.0;
    if (abs_input <= lower_)
    {
        return input; // 安全区
    }
    if (abs_input >= upper_)
    {
        return sign * upper_; // 硬限幅区
    }
    // 计算立方衰减
    const double x = (abs_input - lower_) / (upper_ - lower_); // 归一化
    const double eased = std::pow(x, 3.0);

    return sign * (lower_ + (upper_ - lower_) * eased);
}

LowPassFilter::LowPassFilter(double cutoff_freq, double dt)
    : alpha_(1 - std::exp(-2 * M_PI * cutoff_freq * dt)),
      prev_value_(std::numeric_limits<double>::quiet_NaN())
{
}

double LowPassFilter::update(double new_value)
{
    if (std::isnan(prev_value_))
    {
        prev_value_ = new_value;
        return new_value;
    }
    prev_value_ += alpha_ * (new_value - prev_value_);
    return prev_value_;
}

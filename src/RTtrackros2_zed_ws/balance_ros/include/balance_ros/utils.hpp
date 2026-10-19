#ifndef BALANCE_ROS_UTILS_HPP
#define BALANCE_ROS_UTILS_HPP

#include <cassert>
#include <cmath>

class SoftLimter
{
private:
    const double lower_; // 软限幅起始值
    const double upper_; // 软限幅终止值
    // 有效性验证
    void validateParameters() const;

public:
    SoftLimter(double lower_thres, double upper_thres);
    // 限幅处理
    double clamp(double input) const;
};

class LowPassFilter // 一阶低通滤波
{
private:
    double alpha_;
    double prev_value_;

public:
    LowPassFilter(double cutoff_freq, double dt);
    double update(double new_value);
};
#endif // BALANCE_ROS_UTILS_HPP
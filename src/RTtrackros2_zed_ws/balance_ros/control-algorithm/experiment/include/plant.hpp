#pragma once
#include <chrono>
#include <vector>

class Plant
{
  public:
    virtual ~Plant() = default;
    virtual std::vector<double> System(std::vector<double> state, std::vector<double> control) = 0;
    virtual std::vector<double> GenerateInitialState() = 0;
    virtual std::vector<double> GenerateEmptyControl() = 0;
};
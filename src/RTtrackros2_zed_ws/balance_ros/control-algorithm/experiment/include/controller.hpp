#pragma once
#include <vector>

class Controller
{
  public:
    virtual ~Controller() = default;
    virtual std::vector<double> GetControl(std::vector<double> x) = 0;
};
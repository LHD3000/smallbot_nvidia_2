#pragma once
#include <QWidget>
#include <vector>

class Visualizer
{
  public:
    using StateVector = std::vector<double>;
    using ControlVector = std::vector<double>;

  public:
    virtual ~Visualizer() = default;
    virtual QWidget* MakeUp() = 0;
    virtual void Reset() = 0;
    virtual void Update(StateVector x, ControlVector u) = 0;
};
#pragma once
#include "controller.hpp"
#include "plant.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <functional>
#include <memory>

class Simulator
{
  private:
    std::shared_ptr<Plant> plant;
    std::shared_ptr<Controller> controller;
    std::function<void(const double t, const std::vector<double> &state, const std::vector<double> &control)>
        OnObserver;

  public:
    Simulator(std::shared_ptr<Plant> plant, std::shared_ptr<Controller> controller,
              std::function<void(const double t, const std::vector<double> &state, const std::vector<double> &control)>
                  onObserver);

    void Start(double range, double dt);

    //void Pause();
    //void Resume();
    //void Stop();
};
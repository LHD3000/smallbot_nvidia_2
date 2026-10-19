#include "simulator.hpp"
Simulator::Simulator(
    std::shared_ptr<Plant> plant, std::shared_ptr<Controller> controller,
    std::function<void(const double t, const std::vector<double> &state, const std::vector<double> &control)>
        onObserver)
    : plant(plant), controller(controller), OnObserver(onObserver)
{
}

void Simulator::Start(double range, double dt)
{
    auto state = plant->GenerateInitialState();
    auto control = controller->GetControl(state);
    // warning control must be valid before calling integrate_const
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<std::vector<double>>(),
        [this, &control](const std::vector<double> &nstate, std::vector<double> &dState, const double t)
        { dState = plant->System(nstate, control); },
        state, 0.0, range, dt,
        [this, &control](const std::vector<double> &state, const double t) { OnObserver(t, state, control); });
}

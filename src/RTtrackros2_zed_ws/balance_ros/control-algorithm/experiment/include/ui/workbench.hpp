#pragma once

#include "simpack.hpp"
#include <QSplitter>
#include <QWidget>

#include "ui/loader.hpp"
#include "ui/plant_inspector.hpp"
#include "ui/recorder.hpp"
#include "ui/simulator_controller.hpp"
#include "ui/controller_inspector.hpp"

class Workbench : public QSplitter
{
  private:
    Loader* loader;
    PlantInspector* plantInspector;
    ControllerInspector* controllerInspector;
    SimulatorController* simulatorController;
    Recorder* recorder;
    QWidget* rightColumn;
    std::vector<std::shared_ptr<Visualizer>> visualizers;
  public:
    /*
     * @brief Default constructor
     */
    Workbench(QWidget *parent = nullptr);
    /*
     * @brief Constructor used for code interface
     * @param simPack Simulation package
     */
    Workbench(const SimPack &simPack, QWidget *parent = nullptr);
};
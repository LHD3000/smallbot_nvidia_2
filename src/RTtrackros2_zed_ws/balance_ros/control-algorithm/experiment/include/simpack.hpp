#pragma once
#include "controller.hpp"
#include "plant.hpp"
#include "visualizer.hpp"
#include <vector>
struct VisualizerPack
{
    std::shared_ptr<Visualizer> visualizer;
    std::string name;
};
struct ControllerPack
{
    std::shared_ptr<Controller> controller;
    std::string name;
    std::vector<VisualizerPack> visualizers;
};
struct PlantPack
{
    std::shared_ptr<Plant> plant;
    std::string name;
    std::vector<VisualizerPack> visualizers;
    std::vector<ControllerPack> controllers;
};
struct SimPack
{
  public:
    std::vector<PlantPack> plants;
};

extern "C" SimPack *Create();
extern "C" void Destroy(SimPack *simPack);
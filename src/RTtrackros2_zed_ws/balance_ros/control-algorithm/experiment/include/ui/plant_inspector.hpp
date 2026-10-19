#pragma once
#include "simpack.hpp"
#include "ui/args_inspector.hpp"
#include <QComboBox>
#include <QWidget>
#include <functional>

class PlantInspector : public QWidget
{
  private:
    QComboBox *plantComboBox;
    ArgsInspector *argsInspector;
    std::map<std::string, std::shared_ptr<Plant>> plants;
    std::function<void(std::string)> onChanged;

  public:
    PlantInspector(std::function<void(std::string)> onChanged = nullptr, QWidget *parent = nullptr);
    void SetPlant(std::string name, std::shared_ptr<Plant> plant);
    void SetPlants(const std::vector<PlantPack> &plants);
    std::shared_ptr<Plant> CurrentSelectedPlant();
    void SaveArgs();

  private:
    void UpdatePlantList();
    void UpdatePlantArgs();
};

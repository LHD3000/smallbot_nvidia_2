#include "ui/plant_inspector.hpp"
#include "ui/args_layout.hpp"
#include <iostream>

PlantInspector::PlantInspector(std::function<void(std::string)> onChanged, QWidget *parent) : onChanged(onChanged)
{
    setLayout(new QVBoxLayout(this));
    this->layout()->addWidget(new QLabel("Plant"));
    plantComboBox = new QComboBox();
    this->layout()->addWidget(plantComboBox);
    argsInspector = new ArgsInspector();
    this->layout()->addWidget(argsInspector);
    QObject::connect(plantComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [this](int index)
                     {
                         UpdatePlantArgs();
                         if (this->onChanged)
                             this->onChanged(plantComboBox->currentText().toStdString());
                     });
}

void PlantInspector::SetPlant(std::string name, std::shared_ptr<Plant> plant)
{
    this->plants.clear();
    this->plants[name] = plant;
    UpdatePlantList();
}
void PlantInspector::SetPlants(const std::vector<PlantPack> &plants)
{
    this->plants.clear();
    for (auto &plant : plants)
    {
        this->plants[plant.name] = plant.plant;
    }
    UpdatePlantList();
}

std::shared_ptr<Plant> PlantInspector::CurrentSelectedPlant()
{
    return plants[plantComboBox->currentText().toStdString()];
}

void PlantInspector::SaveArgs()
{
    std::shared_ptr<Plant> p = plants[plantComboBox->currentText().toStdString()];
    auto layout = std::dynamic_pointer_cast<ArgsLayout>(p);
    if (layout)
    {
        layout->PhraseArgs(argsInspector->GetArgs());
    }
}

void PlantInspector::UpdatePlantList()
{
    plantComboBox->clear();
    for (auto &plant : plants)
    {
        plantComboBox->addItem(QString::fromStdString(plant.first));
    }
    plantComboBox->setCurrentIndex(0);
    UpdatePlantArgs();
}

void PlantInspector::UpdatePlantArgs()
{
    argsInspector->Reset();
    std::shared_ptr<Plant> p = plants[plantComboBox->currentText().toStdString()];
    // std::cout << "Type of p: " << typeid(*p).name() << std::endl;
    auto layout = std::dynamic_pointer_cast<ArgsLayout>(p);
    if (layout)
    {
        layout->CreateLayout(*argsInspector);
    }
    else
    {
        argsInspector->AddLabel("none");
    }
}
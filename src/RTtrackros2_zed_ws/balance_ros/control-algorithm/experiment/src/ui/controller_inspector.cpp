#include "ui/controller_inspector.hpp"
#include "ui/args_layout.hpp"
ControllerInspector::ControllerInspector(std::function<void(std::string)> onChanged, QWidget *parent)
    : QWidget(parent), onChanged(onChanged)
{
    setLayout(new QVBoxLayout(this));
    layout()->addWidget(new QLabel("Controller"));
    comboBox = new QComboBox();
    layout()->addWidget(comboBox);
    argsInspector = new ArgsInspector();
    layout()->addWidget(argsInspector);

    QObject::connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     [this](int index)
                     {
                         UpdateControllerArgs();
                         if (this->onChanged)
                             this->onChanged(comboBox->currentText().toStdString());
                     });
}

void ControllerInspector::SetController(std::string name, std::shared_ptr<Controller> controller)
{
    this->controllers.clear();
    this->controllers[name] = controller;
    UpdateControllerList();
}

void ControllerInspector::SetControllers(const std::vector<ControllerPack> &controllers)
{
    this->controllers.clear();
    for (auto &controller : controllers)
    {
        this->controllers[controller.name] = controller.controller;
    }
    UpdateControllerList();
}

std::shared_ptr<Controller> ControllerInspector::CurrentSelectedController()
{
    return controllers[comboBox->currentText().toStdString()];
}

void ControllerInspector::SaveArgs() {

    std::shared_ptr<Controller> p = controllers[comboBox->currentText().toStdString()];
    auto layout = std::dynamic_pointer_cast<ArgsLayout>(p);
    if (layout)
    {
        layout->PhraseArgs(argsInspector->GetArgs());
    }
}

void ControllerInspector::UpdateControllerList()
{
    comboBox->clear();
    for (auto &controller : controllers)
    {
        comboBox->addItem(QString::fromStdString(controller.first));
    }
    comboBox->setCurrentIndex(0);
    UpdateControllerArgs();
}

void ControllerInspector::UpdateControllerArgs()
{
    argsInspector->Reset();
    std::shared_ptr<Controller> c = controllers[comboBox->currentText().toStdString()];
    auto layout = std::dynamic_pointer_cast<ArgsLayout>(c);
    if (layout)
    {
        layout->CreateLayout(*argsInspector);
    }
    else
    {
        argsInspector->AddLabel("none");
    }
}

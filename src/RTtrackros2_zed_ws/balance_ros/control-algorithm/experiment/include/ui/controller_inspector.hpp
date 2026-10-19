#pragma once

#include "args_inspector.hpp"
#include "simpack.hpp"
#include <QComboBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

class ControllerInspector : public QWidget
{
  private:
    QComboBox *comboBox;
    ArgsInspector *argsInspector;
    std::map<std::string, std::shared_ptr<Controller>> controllers;
    std::function<void(std::string)> onChanged;

  public:
    ControllerInspector(std::function<void(std::string)> onChanged, QWidget *parent = nullptr);
    void SetControllers(const std::vector<ControllerPack> &controllers);
    void SetController(std::string name, std::shared_ptr<Controller> controller);
    std::shared_ptr<Controller> CurrentSelectedController();
    void SaveArgs();

  private:
    void UpdateControllerList();
    void UpdateControllerArgs();
};

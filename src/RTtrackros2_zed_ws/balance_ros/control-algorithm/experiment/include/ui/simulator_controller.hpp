#pragma once

#include <QDoubleValidator>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <functional>

class SimulatorController : public QWidget
{
  private:
    double range = 10;
    double simulateRate = 100;
    double controlRate = 50;
    std::function<void(double range, double simRate, double ctrlRate)> onRun;

  public:
    SimulatorController(std::function<void(double range, double simRate, double ctrlRate)> func = nullptr,
                        QWidget *parent = nullptr);
};

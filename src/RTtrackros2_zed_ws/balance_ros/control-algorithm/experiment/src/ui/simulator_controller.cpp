#include "ui/simulator_controller.hpp"
#include <iostream>

SimulatorController::SimulatorController(std::function<void(double, double, double)> func, QWidget *parent)
    : QWidget(parent), onRun(func)
{
    setLayout(new QVBoxLayout());
    layout()->addWidget(new QLabel("Simulator"));

    auto gridLayout = new QGridLayout();
    QLabel *simTimeLabel = new QLabel("time range");
    QLineEdit *simTimeEdit = new QLineEdit(QString::number(range));
    simTimeEdit->setValidator(new QDoubleValidator());
    gridLayout->addWidget(simTimeLabel, 0, 0);
    gridLayout->addWidget(simTimeEdit, 0, 1);
    QObject::connect(simTimeEdit, &QLineEdit::editingFinished,
                     [this, simTimeEdit]() { this->range = simTimeEdit->text().toDouble(); });

    QLabel *simFreqLabel = new QLabel("simulate rate");
    QLineEdit *simFreqEdit = new QLineEdit(QString::number(simulateRate));
    simFreqEdit->setValidator(new QDoubleValidator());
    gridLayout->addWidget(simFreqLabel, 1, 0);
    gridLayout->addWidget(simFreqEdit, 1, 1);
    QObject::connect(simFreqEdit, &QLineEdit::editingFinished,
                     [this, simFreqEdit]() { this->simulateRate = simFreqEdit->text().toDouble(); });

    QLabel *ctrlFreqLabel = new QLabel("control rate");
    QLineEdit *ctrlFreqEdit = new QLineEdit(QString::number(controlRate));
    ctrlFreqEdit->setValidator(new QDoubleValidator());
    gridLayout->addWidget(ctrlFreqLabel, 2, 0);
    gridLayout->addWidget(ctrlFreqEdit, 2, 1);
    QObject::connect(ctrlFreqEdit, &QLineEdit::editingFinished,
                     [this, ctrlFreqEdit]() { this->controlRate = ctrlFreqEdit->text().toDouble(); });

    auto params = new QWidget();
    params->setLayout(gridLayout);
    layout()->addWidget(params);
    auto runButton = new QPushButton("Run");

    layout()->addWidget(runButton);
    QObject::connect(runButton, &QPushButton::clicked,
                     [this]()
                     {
                         if (onRun)
                         {
                             onRun(range, simulateRate, controlRate);
                         }
                     });
}

#pragma once
#include "database.hpp"
#include <QDebug>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <functional>

class Recorder : public QWidget
{
  private:
    QProgressBar *progressBar;
    DataBase dataBase;
    QTimer *timer;
    std::function<void()> onStart;
    std::function<void(DataBase::DataPoint)> onUpdate;
    std::function<void()> onFinish;
    double currentTime = 0;
    double speed = 1;
    double refreshRate = 30;

  public:
    Recorder(QWidget *parent = nullptr);
    DataBase &GetDataBase();
    void SetOnStart(std::function<void()> onStart);
    void SetOnUpdate(std::function<void(DataBase::DataPoint)> onUpdate);
    void SetOnFinish(std::function<void()> onFinish);

  private:
    void OnTimer();
};

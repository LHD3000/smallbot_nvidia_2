#pragma once

#include <QApplication>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "lib_loader.hpp"
#include "simpack.hpp"
#include <functional>

class Loader : public QWidget
{
  private:
    using CreateFunctionType = SimPack *(*)(void);
    using DestroyFunctionType = void (*)(SimPack *);
    std::unique_ptr<SimPack> simPack;
    CreateFunctionType CreateFunction;
    DestroyFunctionType DestroyFunction;
    LibLoader loader;
    std::function<void(const SimPack&)> onLoad;

  public:
    Loader(std::function<void(const SimPack&)> onLoad = nullptr, QWidget *parent = nullptr);
    const SimPack& GetSimPack();
    
  private:
    void LoadLibrary(const QString &fileName);
};

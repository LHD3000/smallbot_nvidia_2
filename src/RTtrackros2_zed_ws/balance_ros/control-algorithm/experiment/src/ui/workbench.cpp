#include "ui/workbench.hpp"
#include "simulator.hpp"
#include "ui/args_layout.hpp"
#include <QLayout>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QThread>
#include <map>

Workbench::Workbench(QWidget *parent) : QSplitter(Qt::Horizontal, parent)
{
    auto leftColumn = new QWidget();
    this->addWidget(leftColumn);
    leftColumn->setLayout(new QVBoxLayout());

    rightColumn = new QWidget();
    rightColumn->setLayout(new QVBoxLayout());
    this->addWidget(rightColumn);

    loader = new Loader(
        [this](const SimPack &simPack)
        {
            std::cout << "Loaded" << std::endl;
            this->plantInspector->SetPlants(simPack.plants);
        });
    plantInspector = new PlantInspector(
        [this](std::string name)
        {
            std::cout << "Plant changed: " << name << std::endl;
            auto &plants = this->loader->GetSimPack().plants;
            auto it = std::find_if(plants.begin(), plants.end(),
                                   [&name](const PlantPack &plant) { return plant.name == name; });
            if (it != plants.end())
            {
                controllerInspector->SetControllers(it->controllers);

                // set visualizers
                // remove old
                QLayoutItem *child;
                while ((child = this->rightColumn->layout()->takeAt(0)) != nullptr)
                {
                    delete child->widget();
                    delete child;
                }
                visualizers.clear();
                // add new
                for (auto visualizer : it->visualizers)
                {
                    auto vi = visualizer.visualizer->MakeUp();
                    if (vi != nullptr)
                    {
                        this->visualizers.push_back(visualizer.visualizer);
                        this->rightColumn->layout()->addWidget(vi);
                    }
                }
            }
        });
    controllerInspector =
        new ControllerInspector([this](std::string name) { std::cout << "Controller changed: " << name << std::endl; });
    simulatorController = new SimulatorController(
        [this](double range, double simRate, double ctrlRate)
        {
            std::cout << "Simulator start" << range << " " << simRate << " " << ctrlRate << std::endl;
            QThread *simThread = QThread::create(
                [this, range, simRate, ctrlRate]()
                {
                    for (auto vi : visualizers)
                    {
                        vi->Reset();
                    }
                    this->recorder->GetDataBase().Clear();
                    this->plantInspector->SaveArgs();
                    this->controllerInspector->SaveArgs();
                    Simulator(this->plantInspector->CurrentSelectedPlant(),
                              this->controllerInspector->CurrentSelectedController(),
                              [this, simRate](const double t, const std::vector<double> &state,
                                              const std::vector<double> &control)
                              {
                                  QMetaObject::invokeMethod(this,
                                                            [this, t, state, control]()
                                                            {
                                                                this->recorder->GetDataBase().SaveData({t, state, control});
                                                                for (auto vi : visualizers)
                                                                {
                                                                    vi->Update(state, control);
                                                                }
                                                            });
                                  QThread::msleep(1000 / simRate);
                              })
                        .Start(range, 1.0 / ctrlRate);
                });
            simThread->start();
        });

    recorder = new Recorder();
    recorder->SetOnStart(
        [this]()
        {
            // std::cout << "Start" << std::endl;
            for (auto vi : visualizers)
            {
                vi->Reset();
            }
        });
    recorder->SetOnUpdate(
        [this](DataBase::DataPoint data)
        {
            // std::cout << "Update" << std::endl;
            for (auto vi : visualizers)
            {
                auto [time, state, control] = data;
                vi->Update(state, control);
            }
        });
    recorder->SetOnFinish([this]() { std::cout << "Record Finished" << std::endl; });

    QScrollArea *scrollArea = new QScrollArea();
    scrollArea->setWidgetResizable(true);

    QWidget *scrollContent = new QWidget();
    QVBoxLayout *scrollLayout = new QVBoxLayout();
    scrollContent->setLayout(scrollLayout);

    scrollLayout->addWidget(loader);
    scrollLayout->addWidget(plantInspector);
    scrollLayout->addWidget(controllerInspector);
    scrollLayout->addWidget(simulatorController);
    scrollLayout->addWidget(recorder);
    scrollLayout->addStretch();

    scrollArea->setWidget(scrollContent);
    leftColumn->layout()->addWidget(scrollArea);

    this->setStretchFactor(0, 0);
    this->setStretchFactor(1, 1);
    setOpaqueResize(true);
}

Workbench::Workbench(const SimPack &simPack, QWidget *parent)
{
    // auto leftColumn = new QWidget();
    // this->addWidget(leftColumn);
    // leftColumn->setLayout(new QVBoxLayout());
    //
    // auto rightColumn = new QWidget();
    // this->addWidget(rightColumn);
    // rightColumn->setLayout(new QVBoxLayout());
    //
    // auto viewer = new Viewer2D();
    // auto figure = new Figure();
    //
    // auto plantInspector = new PlantInspector();
    // auto controllerInspector = new ControllerInspector();
    // auto simulatorController = new SimulatorController();
    // auto recorder = new Recorder();
    //
    // QScrollArea *scrollArea = new QScrollArea();
    // scrollArea->setWidgetResizable(true);
    //
    // QWidget *scrollContent = new QWidget();
    // QVBoxLayout *scrollLayout = new QVBoxLayout();
    // scrollContent->setLayout(scrollLayout);
    //
    // scrollLayout->addWidget(plantInspector);
    // scrollLayout->addWidget(controllerInspector);
    // scrollLayout->addWidget(simulatorController);
    // scrollLayout->addWidget(recorder);
    //
    // scrollArea->setWidget(scrollContent);
    // leftColumn->layout()->addWidget(scrollArea);
    //
    // rightColumn->layout()->addWidget(viewer);
    // rightColumn->layout()->addWidget(figure);
    //
    // this->setStretchFactor(0, 0);
    // this->setStretchFactor(1, 1);
    // setOpaqueResize(true);
}
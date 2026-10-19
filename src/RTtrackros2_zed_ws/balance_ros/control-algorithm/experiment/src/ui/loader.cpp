#include "ui/loader.hpp"
#include <QMessageBox>

Loader::Loader(std::function<void(const SimPack &)> onLoad, QWidget *parent) : QWidget(parent), onLoad(onLoad)
{
    if (onLoad == nullptr)
    {
        this->setEnabled(false);
    }
    QVBoxLayout *layout = new QVBoxLayout();
    setLayout(layout);
    layout->addWidget(new QLabel("Loader"));

    auto explore = new QPushButton("Explore");
    layout->addWidget(explore);

    auto selectedLable = new QLabel("Unselected");
    layout->addWidget(selectedLable);

    connect(explore, &QPushButton::clicked,
            [this, selectedLable]()
            {
                QString executablePath = QApplication::applicationDirPath();
                QString fileName = QFileDialog::getOpenFileName(this, tr("Open Dynamic Link Library"), executablePath,
                                                                tr("Dynamic Link Libraries (*.so *.dll)"));
                if (!fileName.isEmpty())
                {
                    QFileInfo fileInfo(fileName);
                    QString baseName = fileInfo.baseName();
                    selectedLable->setText(baseName);
                    LoadLibrary(fileName);
                }
            });
}

const SimPack &Loader::GetSimPack()
{
    return *simPack;
}

void Loader::LoadLibrary(const QString &fileName)
{
    if (simPack != nullptr)
    {
        DestroyFunction(simPack.get());
    }
    try
    {
        loader.Load(fileName.toStdString());
        CreateFunction = reinterpret_cast<CreateFunctionType>(loader.GetFunction("Create"));
        DestroyFunction = reinterpret_cast<DestroyFunctionType>(loader.GetFunction("Destroy"));
    }
    catch (const std::exception &e)
    {
        QMessageBox::critical(this, "Load lib and functions fail", e.what());
    }
    try
    {
        simPack = std::unique_ptr<SimPack>(CreateFunction());
    }
    catch(const std::exception &e)
    {
        QMessageBox::critical(this, "Possible lib has error", e.what());
    }
    if (onLoad)
    {
        onLoad(*simPack);
    }
}

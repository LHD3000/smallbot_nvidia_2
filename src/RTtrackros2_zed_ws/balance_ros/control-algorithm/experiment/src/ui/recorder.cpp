#include "ui/recorder.hpp"
#include <QFileDialog>
#include <QTimer>
#include <iostream>

Recorder::Recorder(QWidget *parent) : QWidget(parent)
{
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Recorder::OnTimer);
    setLayout(new QVBoxLayout());
    layout()->addWidget(new QLabel("Recorder"));

    QGridLayout *layout = new QGridLayout();

    QPushButton *loadButton = new QPushButton("Load");
    QPushButton *saveButton = new QPushButton("Save");
    connect(loadButton, &QPushButton::clicked,
            [this]()
            {
                QString fileName = QFileDialog::getOpenFileName(this, "Open CSV File", "", "CSV Files (*.csv)");
                if (!fileName.isEmpty())
                {
                    dataBase.LoadFile(fileName.toStdString());
                }
            });

    connect(saveButton, &QPushButton::clicked,
            [this]()
            {
                QString fileName = QFileDialog::getSaveFileName(this, "Save CSV File", "", "CSV Files (*.csv)");
                if (!fileName.isEmpty())
                {
                    if (!fileName.endsWith(".csv", Qt::CaseInsensitive))
                    {
                        fileName += ".csv";
                    }
                    dataBase.SaveFile(fileName.toStdString());
                }
            });
    layout->addWidget(loadButton, 0, 0);
    layout->addWidget(saveButton, 0, 1);

    QLabel *DataInfoLabel = new QLabel("Data Info");
    layout->addWidget(DataInfoLabel, 1, 0);
    QLabel *DataInfoValue = new QLabel("3s=120ps");
    layout->addWidget(DataInfoValue, 1, 1);
    QTimer *infoTimer = new QTimer(this);
    connect(infoTimer, &QTimer::timeout,
            [this, DataInfoValue]()
            {
                auto [endTime, size] = dataBase.GetDataInfo();
                DataInfoValue->setText(QString("%1s=%2ps").arg(endTime).arg(size));
            });
    infoTimer->start(1000);

    QLabel *playSpeedLabel = new QLabel("Play Speed");
    layout->addWidget(playSpeedLabel, 2, 0);
    QLineEdit *playSpeedEdit = new QLineEdit("1.0");
    playSpeedEdit->setValidator(new QDoubleValidator());
    layout->addWidget(playSpeedEdit, 2, 1);
    connect(playSpeedEdit, &QLineEdit::textChanged,
            [this](const QString &text)
            {
                if (text.isEmpty())
                {
                    return;
                }
                double val = text.toDouble();
                if (val > 0)
                {
                    this->speed = val;
                }
            });

    progressBar = new QProgressBar();
    progressBar->setRange(0, 100);
    layout->addWidget(progressBar, 2, 0, 1, 4);

    QPushButton *playButton = new QPushButton("Play");
    QPushButton *pauseButton = new QPushButton("Pause");
    QPushButton *resetButton = new QPushButton("Reset");
    connect(playButton, &QPushButton::clicked,
            [this]()
            {
                if (currentTime >= std::get<0>(dataBase.GetDataInfo()))
                {
                    currentTime = 0;
                }
                timer->start(1000 / refreshRate * speed);
            });
    connect(pauseButton, &QPushButton::clicked, [this]() { timer->stop(); });
    connect(resetButton, &QPushButton::clicked,
            [this]()
            {
                timer->stop();
                currentTime = 0;
            });

    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(playButton);
    buttonLayout->addWidget(pauseButton);
    buttonLayout->addWidget(resetButton);
    layout->addLayout(buttonLayout, 3, 0, 1, 4);

    QWidget *params = new QWidget();
    params->setLayout(layout);
    this->layout()->addWidget(params);
}

void Recorder::OnTimer()
{
    if (currentTime == 0)
    {
        if (onStart)
        {
            onStart();
        }
    }
    auto [endTime, size] = dataBase.GetDataInfo();
    currentTime += 1.0 / refreshRate * speed;
    // std::cout << "Current Time: " << currentTime << std::endl;
    if (currentTime >= endTime)
    {
        progressBar->setValue(100);
        timer->stop();
        if (onFinish)
        {
            onFinish();
        }
    }
    progressBar->setValue(currentTime / endTime * 100);
    if (onUpdate)
    {
        onUpdate(dataBase.SamplerData(currentTime));
    }
}

DataBase &Recorder::GetDataBase()
{
    return dataBase;
}

void Recorder::SetOnStart(std::function<void()> onStart)
{
    this->onStart = onStart;
}
void Recorder::SetOnUpdate(std::function<void(DataBase::DataPoint)> onUpdate)
{
    this->onUpdate = onUpdate;
}
void Recorder::SetOnFinish(std::function<void()> onFinish)
{
    this->onFinish = onFinish;
}
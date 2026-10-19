#include "ui/args_inspector.hpp"

// ...existing code...
ArgsInspector::ArgsInspector(QWidget *parent) : QWidget(parent)
{
    layout = new QGridLayout();
    setLayout(layout);
    row = 0;
}

const std::map<std::string, ArgsInspector::Arg> &ArgsInspector::GetArgs()
{
    return args;
}

void ArgsInspector::Reset()
{
    QLayoutItem *item;
    while ((item = layout->takeAt(0)) != nullptr)
    {
        if (item->widget())
        {
            delete item->widget();
        }
        delete item;
    }
    row = 0;
    args.clear();
}

void ArgsInspector::AddLabel(const std::string &label)
{
    QLabel *labelWidget = new QLabel(QString::fromStdString(label), this);
    layout->addWidget(labelWidget, row, 0);
    row++;
}

void ArgsInspector::AddIntInput(const std::string &label, int def)
{
    args[label] = def;
    QLabel *labelWidget = new QLabel(QString::fromStdString(label), this);
    QLineEdit *input = new QLineEdit(QString::number(def), this);
    input->setValidator(new QIntValidator());
    layout->addWidget(labelWidget, row, 0);
    layout->addWidget(input, row, 1);
    QObject::connect(input, &QLineEdit::editingFinished,
                     [this, labelWidget, input]()
                     {
                         int value = input->text().toInt();
                         args[labelWidget->text().toStdString()] = value;
                     });
    row++;
}

void ArgsInspector::AddDoubleInput(const std::string &label, double def)
{
    args[label] = def;
    QLabel *labelWidget = new QLabel(QString::fromStdString(label), this);
    QLineEdit *input = new QLineEdit(QString::number(def), this);
    input->setValidator(new QDoubleValidator());
    layout->addWidget(labelWidget, row, 0);
    layout->addWidget(input, row, 1);
    connect(input, &QLineEdit::editingFinished,
            [this, labelWidget, input]()
            {
                double value = input->text().toDouble();
                args[labelWidget->text().toStdString()] = value;
            });
    row++;
}

void ArgsInspector::AddBoolToggle(const std::string &label, bool def)
{
    args[label] = def;
    QLabel *labelWidget = new QLabel(QString::fromStdString(label), this);
    QCheckBox *input = new QCheckBox(this);
    input->setChecked(def);
    layout->addWidget(labelWidget, row, 0);
    layout->addWidget(input, row, 1);
    connect(input, &QCheckBox::stateChanged,
            [this, labelWidget, input](int state)
            {
                bool value = (state == Qt::Checked);
                args[labelWidget->text().toStdString()] = value;
            });
    row++;
}

void ArgsInspector::AddStringInput(const std::string &label, const std::string &def)
{
    args[label] = def;
    QLabel *labelWidget = new QLabel(QString::fromStdString(label), this);
    QLineEdit *input = new QLineEdit(QString::fromStdString(def), this);
    layout->addWidget(labelWidget, row, 0);
    layout->addWidget(input, row, 1);
    connect(input, &QLineEdit::editingFinished,
            [this, labelWidget, input]()
            {
                std::string value = input->text().toStdString();
                args[labelWidget->text().toStdString()] = value;
            });
    row++;
}

void ArgsInspector::AddEnumSelector(const std::string &label, const std::vector<std::string> &selections, int def)
{
    args[label] = def;
    QLabel *labelWidget = new QLabel(QString::fromStdString(label), this);
    QComboBox *input = new QComboBox(this);
    for (const auto &selection : selections)
    {
        input->addItem(QString::fromStdString(selection));
    }
    input->setCurrentIndex(def);
    layout->addWidget(labelWidget, row, 0);
    layout->addWidget(input, row, 1);
    connect(input, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, labelWidget, input](int index)
            {
                std::string value = input->itemText(index).toStdString();
                args[labelWidget->text().toStdString()] = value;
            });
    row++;
}

void ArgsInspector::TestCreat()
{
    AddIntInput("int", 100);
    AddDoubleInput("double", 100.0);
    AddStringInput("string", "");
    AddBoolToggle("bool", true);
    AddEnumSelector("enum", std::vector<std::string>{"a", "b", "c"});
}

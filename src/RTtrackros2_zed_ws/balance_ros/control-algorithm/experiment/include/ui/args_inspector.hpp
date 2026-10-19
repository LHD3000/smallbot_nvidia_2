#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <variant>
#include <map>
#include <vector>
#include <string>

class ArgsInspector : public QWidget
{
  public:
    using Arg = std::variant<int, double, bool, std::string>;

  private:
    int row;
    QGridLayout *layout;
    std::map<std::string, Arg> args;

  public:
    ArgsInspector(QWidget *parent = nullptr);
    const std::map<std::string, Arg> &GetArgs();
    void Reset();
    void AddLabel(const std::string &label);
    void AddIntInput(const std::string &label, int def);
    void AddDoubleInput(const std::string &label, double def);
    void AddBoolToggle(const std::string &label, bool def);
    void AddStringInput(const std::string &label, const std::string &def = "");
    void AddEnumSelector(const std::string &label, const std::vector<std::string> &selections, int def = 0);
    void TestCreat();
};

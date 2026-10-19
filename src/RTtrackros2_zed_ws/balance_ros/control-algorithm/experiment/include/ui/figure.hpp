#pragma once
#include <QWidget>
#include <QtCharts>
#include <map>
#include <string>
#include <thread>
#include <vector>

class Figure : public QWidget
{
  private:
    std::string title;
    std::string x_label;
    std::string y_label;
    double x_range[2];
    double y_range[2];
    bool legend;
    QtCharts::QChart chart;
    std::map<std::string, QtCharts::QLineSeries *> series;

  public:
    Figure();
    void SetTitle(std::string title);
    void SetLableX(std::string lable);
    void SetLableY(std::string lable);
    void SetRangeX(double x_min, double x_max);
    void SetRangeY(double y_min, double y_max);
    void EnableLegend(bool enable);
    void Plot(std::vector<double> x, std::vector<double> y, std::string label = "");
    void Plot(std::vector<double> y, std::string label = "");
    void Plot(std::string lable);
    void Append(std::string lable, double y);
    void Append(std::string lable, double x, double y);
    void Clear();

  private:
};
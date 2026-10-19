#include "ui/figure.hpp"
#include <QApplication>
#include <QWidget>
#include <QtCharts/QChartView>
#include <QtCharts/QLegend>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <qdebug.h>
#include <qlegendmarker.h>
#include <vector>

Figure::Figure() : title(""), x_label("x"), y_label("y"), legend(false)
{
    this->x_range[0] = 0;
    this->x_range[1] = 0;
    this->y_range[0] = 0;
    this->y_range[1] = 0;

    // OP chart
    chart.setTitle(QString::fromStdString(this->title));
    auto axisX = new QValueAxis();
    auto axisY = new QValueAxis();
    chart.addAxis(axisX, Qt::AlignBottom);
    chart.addAxis(axisY, Qt::AlignLeft);
    chart.axes()[0]->setTitleText(QString::fromStdString(x_label));
    chart.axes()[1]->setTitleText(QString::fromStdString(y_label));
    if (x_range[0] != x_range[1])
    {
        chart.axes()[0]->setRange(x_range[0], x_range[1]);
    }
    if (y_range[0] != y_range[1])
    {
        chart.axes()[1]->setRange(y_range[0], y_range[1]);
    }
    chart.legend()->setVisible(legend);

    QtCharts::QChartView *chartView = new QtCharts::QChartView(&chart, this);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setWindowTitle(QString::fromStdString(title));

    // layout
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->addWidget(chartView);
    this->setLayout(layout);
    this->resize(600, 400);
}
void Figure::SetTitle(std::string title)
{
    this->title = title;
}
void Figure::SetLableX(std::string lable)
{
    this->x_label = lable;
};
void Figure::SetLableY(std::string lable)
{
    this->y_label = lable;
};
void Figure::SetRangeX(double x_min, double x_max)
{
    this->x_range[0] = x_min;
    this->x_range[1] = x_max;
}

void Figure::SetRangeY(double y_min, double y_max)
{
    this->y_range[0] = y_min;
    this->y_range[1] = y_max;
}

void Figure::EnableLegend(bool enable)
{
    this->legend = enable;
    chart.legend()->setVisible(legend);
}
void Figure::Plot(std::vector<double> x, std::vector<double> y, std::string label)
{
    QtCharts::QLineSeries *lineSeries = new QtCharts::QLineSeries(this);
    for (int i = 0; i < y.size(); i++)
    {
        lineSeries->append(x[i], y[i]);
    }
    lineSeries->setName(QString::fromStdString(label));
    chart.addSeries(lineSeries);
    series[label] = lineSeries;
    lineSeries->attachAxis(chart.axes()[0]);
    lineSeries->attachAxis(chart.axes()[1]);
}
void Figure::Plot(std::vector<double> y, std::string label)
{
    std::vector<double> x(y.size());
    for (int i = 0; i < y.size(); i++)
    {
        x[i] = i;
    }
    this->Plot(x, y, label);
}

void Figure::Plot(std::string lable)
{
    this->Plot(std::vector<double>(), std::vector<double>(), lable);
}

void Figure::Append(std::string lable, double y)
{
    if (series.find(lable) == series.end())
        throw std::runtime_error("No such lable: " + lable);
    int x = series[lable]->count();
    series[lable]->append(x, y);
    chart.axes()[0]->setMax(x);
    auto yaix = dynamic_cast<QValueAxis *>(chart.axes()[1]);
    chart.axes()[1]->setRange(std::min(y, yaix->min()), std::max(y, yaix->max()));
}
void Figure::Append(std::string lable, double x, double y)
{
    if (series.find(lable) == series.end())
        throw std::runtime_error("No such lable: " + lable);
    series[lable]->append(x, y);
    chart.axes()[0]->setMax(x);
    auto yaix = dynamic_cast<QValueAxis *>(chart.axes()[1]);
    chart.axes()[1]->setRange(std::min(y, yaix->min()), std::max(y, yaix->max()));
}

void Figure::Clear()
{
    chart.removeAllSeries();
    series.clear();
}

#include "ui/figure.hpp"
#include <iostream>
#include <math.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Figure fig;
    fig.SetTitle("draw example");
    fig.SetLableX("x");
    fig.SetLableY("y");
    std::vector<double> values1, values2;
    for (int i = 0; i < 100; i++)
    {
        values1.push_back(std::sin(i / 10.0f));
        values2.push_back(std::cos(i / 10.0f));
    }
    fig.Plot(values1, "v1");
    fig.Plot(values2, "v2");
    fig.Plot(values1, values2, "circle");
    fig.EnableLegend(true);
    fig.show();
    return a.exec();
}
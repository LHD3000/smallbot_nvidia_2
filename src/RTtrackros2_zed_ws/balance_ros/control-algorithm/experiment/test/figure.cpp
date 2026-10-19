#include "visualizer.hpp"
#include <memory>
#include <ui/figure.hpp>
#include <ui/viewer_2d.hpp>

class FigureVisualizer : public Visualizer
{
  private:
    Figure *figure;

  public:
    QWidget *MakeUp() override
    {
        figure = new Figure();
        figure->Plot("distance");
        figure->Plot("velocity");
        figure->Plot("angle");
        figure->Plot("角速度");
        figure->Plot("control");
        figure->EnableLegend(true);
        return figure;
    }
    void Reset() override { figure->Clear(); }
    void Update(StateVector x, ControlVector u) override
    {
        figure->Append("distance", x[0]);
        figure->Append("velocity", x[1]);
        figure->Append("angle", x[2]);
        figure->Append("角速度", x[3]);
        figure->Append("control", u[0]);
    }
};

int main(int argc, char **argv)
{

    QApplication app(argc, argv);
    auto figure = new Figure();
    figure->Plot("distance");
    figure->Plot("velocity");
    figure->Plot("angle");
    figure->Plot("角速度");
    figure->Plot("control");
    figure->EnableLegend(true);

    for (int i = 0; i < 100; i++)
    {
        figure->Append("distance", 1);
        figure->Append("velocity", 2);
        figure->Append("angle", 3);
        figure->Append("角速度", 4);
        figure->Append("control", 5);
    }
    figure->show();

    return app.exec();
}

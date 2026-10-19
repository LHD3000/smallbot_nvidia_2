#include "simpack.hpp"
#include "ui/args_layout.hpp"
#include <ctrl_alg/pid/pid_pos.hpp>
#include <eigen3/Eigen/Dense>
#include <math.h>

class InvertedPendulumPlant : public Plant, public ArgsLayout
{
  private:
    double m1 = 1;
    double m2 = 1;
    double I = 1;
    double g = 9.8;
    double maxF = 100;
    double l = 1;
    double initAngle = 0;

  public:
    InvertedPendulumPlant() {}
    InvertedPendulumPlant(double m1, double m2, double I, double maxF, double l)
        : m1(m1), m2(m2), I(I), maxF(maxF), l(l)
    {
    }
    // control = [horizental-force]
    std::vector<double> GenerateEmptyControl() override { return {0.0}; }
    // state = [s, ds, theta, dtheta]
    std::vector<double> GenerateInitialState() override { return {0.0, 0.0, initAngle, 0.0}; }
    std::vector<double> System(std::vector<double> state, std::vector<double> control) override
    {
        double s = state[0];
        double dS = state[1];
        double Theta = state[2];
        double dTheta = state[3];
        double force = control[0];

        Eigen::Matrix2d A;
        A << m1 + m2, m1 * l * cos(Theta), m1 * l * cos(Theta), m1 * l * l + I;
        Eigen::Vector2d Y;
        Y << force + m1 * dTheta * dTheta * l * sin(Theta), m1 * g * l * sin(Theta);
        Eigen::Vector2d result = A.householderQr().solve(Y);

        return std::vector<double>{dS, result[0], dTheta, result[1]};
    }

    void CreateLayout(ArgsInspector &args_inspector) override
    {
        args_inspector.AddDoubleInput("mass-stick", 2);
        args_inspector.AddDoubleInput("mass-box", 1);
        args_inspector.AddDoubleInput("inertia", 0.6);
        args_inspector.AddDoubleInput("length", 10);
        args_inspector.AddDoubleInput("init", 0);
    }

    void PhraseArgs(const std::map<std::string, ArgsInspector::Arg> &args) override
    {
        m1 = std::get<double>(args.at("mass-stick"));
        m2 = std::get<double>(args.at("mass-box"));
        I = std::get<double>(args.at("inertia"));
        l = std::get<double>(args.at("length"));
        initAngle = std::get<double>(args.at("init")) / 180 * std::numbers::pi;
    }
};

class PIDController : public Controller, public ArgsLayout
{
  private:
    CtrlAlg::PIDPos<double> pid;

  public:
    PIDController(double kp, double ki, double kd) : pid(kp, ki, kd) {}
    std::vector<double> GetControl(std::vector<double> state) override
    {
        double angle = fmod(state[2], std::numbers::pi * 2);
        angle = angle > std::numbers::pi ? angle - std::numbers::pi * 2 : angle;
        angle = angle < -std::numbers::pi ? angle + std::numbers::pi * 2 : angle;

        double force = pid.GetControl(angle);
        return std::vector<double>{force};
    }
    void CreateLayout(ArgsInspector &args_inspector) override
    {
        args_inspector.AddDoubleInput("kp", 0.0);
        args_inspector.AddDoubleInput("ki", 0.0);
        args_inspector.AddDoubleInput("kd", 0.0);
    }

    void PhraseArgs(const std::map<std::string, ArgsInspector::Arg> &args) override
    {
        pid.SetP(std::get<double>(args.at("kp")));
        pid.SetI(std::get<double>(args.at("ki")));
        pid.SetD(std::get<double>(args.at("kd")));
    }
};

#include <QGraphicsItem>
#include <ui/figure.hpp>
#include <ui/viewer_2d.hpp>
#include <visualizer.hpp>

class ViewerVisualizer : public Visualizer
{
  public:
    class Viewer : public Viewer2D::Viewer
    {
      private:
        QGraphicsRectItem *cube;
        QGraphicsRectItem *link;

      public:
        void MakeUp(QGraphicsScene &scene) override
        {
            cube = scene.addRect(QRectF(-20, -10, 40, 20));
            link = scene.addRect(QRectF(-5, -100, 10, 100));
            cube->setPen(QPen(Qt::black));
            cube->setBrush(QBrush(Qt::red));
            link->setPen(QPen(Qt::black));
            link->setBrush(QBrush(Qt::blue));
            link->setParentItem(cube);
            link->setTransform(QTransform());
        }
        void Update(std::vector<double> state) override
        {
            cube->setPos(state[0], 0);
            link->setRotation(state[2] / std::numbers::pi * 180);
        }
    };

  private:
    Viewer2D *viewer;

  public:
    QWidget *MakeUp() override
    {
        viewer = new Viewer2D();
        viewer->AddRobot(std::make_shared<Viewer>());
        viewer->AddGround(5, 1000);
        return viewer;
    }
    void Reset() override {}
    void Update(StateVector x, ControlVector u) override { viewer->UpdateStateSpace(x); }
};

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
    void Reset() override
    {
        // figure->Clear();
    }
    void Update(StateVector x, ControlVector u) override
    {
        figure->Append("distance", x[0]);
        figure->Append("velocity", x[1]);
        figure->Append("angle", x[2] / std::numbers::pi * 180);
        figure->Append("角速度", x[3] / std::numbers::pi * 180);
        figure->Append("control", u[0]);
    }
};

extern "C" SimPack *Create()
{
    return new SimPack{.plants = {PlantPack{
                           .plant = std::make_shared<InvertedPendulumPlant>(),
                           .name = "Pendulum",
                           .visualizers{
                               VisualizerPack{
                                   .visualizer = std::make_shared<ViewerVisualizer>(),
                                   .name = "Viewer",
                               },
                               VisualizerPack{
                                   .visualizer = std::make_shared<FigureVisualizer>(),
                                   .name = "Figure",
                               },
                           },
                           .controllers =
                               {
                                   ControllerPack{
                                       .controller = std::make_shared<PIDController>(50, 0, 100),
                                       .name = "PIDPos",
                                   },
                               },
                       }}};
}

extern "C" void Destroy(SimPack *simPack)
{
    delete simPack;
}
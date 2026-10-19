#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPoint>
#include <QWidget>

class Viewer2D : public QWidget
{
  public:
    class Viewer
    {
      public:
        using StateVector = std::vector<double>;

      public:
        virtual ~Viewer() = default;
        virtual void MakeUp(QGraphicsScene &scene) = 0;
        virtual void Update(StateVector x) = 0;
    };

  private:
    QGraphicsScene scene;
    QGraphicsView view;
    std::vector<std::shared_ptr<Viewer>> robots;

  public:
    Viewer2D(QWidget *parent = nullptr);
    void AddGround(float height, float width);
    void AddGrid(float size, float width, float height);
    void AddRobot(std::shared_ptr<Viewer> robot);
    void UpdateStateSpace(std::vector<double> x);

  private:
    virtual void wheelEvent(QWheelEvent *event) override;
};

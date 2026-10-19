#include "ui/viewer_2d.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QGraphicsItem>
#include <QLayout>
#include <QMouseEvent>
#include <QTimer>
#include <QWheelEvent>
#include <iostream>
#include <math.h>
#include <thread>

Viewer2D::Viewer2D(QWidget *parent) : QWidget(parent)
{
    QHBoxLayout *layout = new QHBoxLayout();
    this->setLayout(layout);
    view.setScene(&scene);
    view.resize(800, 600);
    // view.show();
    layout->addWidget(&view);
    view.setMouseTracking(true);
}

void Viewer2D::AddGrid(float size, float width, float height)
{
    QGraphicsRectItem *item = new QGraphicsRectItem();
    item->setRect(QRect(-width / 2, -height / 2, width, height));
    QTransform tf;
    QBrush brush(Qt::CrossPattern);
    tf.scale(size / 4, size / 4);
    brush.setTransform(tf);
    brush.setColor(Qt::gray);
    item->setBrush(brush);
    scene.addItem(item);
}

void Viewer2D::AddGround(float height, float width)
{
    QGraphicsRectItem *item = new QGraphicsRectItem();
    item->setRect(-width / 2, 0, width, height);
    item->setBrush(Qt::BDiagPattern);
    scene.addItem(item);
}

void Viewer2D::AddRobot(std::shared_ptr<Viewer2D::Viewer> robot)
{
    robots.push_back(robot);
    robot->MakeUp(scene);
}

void Viewer2D::UpdateStateSpace(std::vector<double> x)
{
    for (auto robot : robots)
    {
        robot->Update(x);
    }
}

void Viewer2D::wheelEvent(QWheelEvent *event)
{
    float scale = 1.0f + event->angleDelta().y() / 1200.0f;
    view.scale(scale, scale);
}
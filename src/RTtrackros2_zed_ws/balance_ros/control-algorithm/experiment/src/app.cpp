#include "ui/workbench.hpp"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Workbench w;
    w.show();
    return app.exec();
}
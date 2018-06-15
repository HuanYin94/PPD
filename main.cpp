#include "pathplanningdemonstrator.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PathPlanningDemonstrator w;
    w.show();

    return a.exec();
}

#ifndef PATHPLANNINGDEMONSTRATOR_H
#define PATHPLANNINGDEMONSTRATOR_H

#include <QMainWindow>

#include "map.h"
#include "pathplanner.h"

namespace Ui {
    class PathPlanningDemonstrator;
}

class PathPlanningDemonstrator : public QMainWindow
{
    Q_OBJECT

public:
    explicit PathPlanningDemonstrator(QWidget *parent = 0);
    ~PathPlanningDemonstrator();

private:
    void addPlanner(PathPlanner* planner);

private slots:
    void on_actionLoad_triggered();

    void on_plan_utton_clicked();

    void on_planner_box_currentIndexChanged(int index);

private:
    Ui::PathPlanningDemonstrator *ui;

    Map *map_;

    std::vector<PathPlanner *> planners_;
};

#endif // PATHPLANNINGDEMONSTRATOR_H

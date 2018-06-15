#ifndef ASTARPATHPLANNER_H
#define ASTARPATHPLANNER_H

#include "gridpathplanner.h"

class AStarPathPlanner : public GridPathPlanner
{
public:
    AStarPathPlanner();

    QString getName() const;

protected:
    bool plan(
            const QPoint &source,
            const QPoint &terminal,
            const GridMap &grid_map,
            const int rows,
            const int cols,
            QGraphicsScene *scene,
            PathI &path);
};

#endif // ASTARPATHPLANNER_H

#ifndef DIJKSTRAPATHPLANNER_H
#define DIJKSTRAPATHPLANNER_H

#include "gridpathplanner.h"

class DijkstraPathPlanner : public GridPathPlanner
{
public:
    DijkstraPathPlanner();

    QString getName() const;

protected:
    bool plan(const QPoint &source,
              const QPoint &terminal,
              const GridMap &grid_map,
              const int rows,
              const int cols,
              QGraphicsScene *scene,
              PathI &path);
};

#endif // DIJKSTRAPATHPLANNER_H

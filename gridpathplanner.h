#ifndef GRIDPATHPLANNER_H
#define GRIDPATHPLANNER_H

#include "pathplanner.h"
#include "config.h"

class GridPathPlanner : public PathPlanner
{
protected:

    typedef std::vector<std::vector<bool> > GridMap;

    typedef std::vector<QPoint> PathI;

public:
    GridPathPlanner();

    bool plan(const QPointF &source,
         const QPointF &terminal,
         const std::vector<QPolygonF> &polygons,
         QGraphicsScene *scene,
         Path &path);

protected:
    virtual bool plan(const QPoint &source,
                      const QPoint &terminal,
                      const GridMap &grid_map,
                      const int rows,
                      const int cols,
                      QGraphicsScene *scene,
                      PathI &path) = 0;
private:

    void gridize(const QPointF &source_in,
                 const QPointF &terminal_in,
                 const std::vector<QPolygonF> &obstacles,
                 QPoint &source_out,
                 QPoint &terminal_out,
                 GridMap &grid_map,
                 int &rows,
                 int &cols);

//    bool pointInPolygon(const QPointF &point, const QPolygonF &polygon);

private:
    double min_x_, min_y_, max_x_, max_y_;
};

#endif // GRIDPATHPLANNER_H

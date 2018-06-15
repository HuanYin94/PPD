#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>

#include <QPointF>
#include <QPolygonF>
#include <QGraphicsScene>

class PathPlanner
{
public:
    typedef std::vector<QPointF> Path;

    PathPlanner();

    virtual bool plan(const QPointF &source,
              const QPointF &terminal,
              const std::vector<QPolygonF> &polygons,
              QGraphicsScene *scene,
              Path &path) = 0;

    virtual QString getName() const = 0;

protected:

    bool pointInPolygon(const QPointF &point, const QPolygonF &polygon);

    double distance(const QPoint &p1, const QPoint &p2);

    double distance(const QPointF &p1, const QPointF &p2);

};

#endif // PATHPLANNER_H

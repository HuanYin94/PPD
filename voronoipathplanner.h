#ifndef VORONOIPATHPLANNER_H
#define VORONOIPATHPLANNER_H

#include "polygonpathplanner.h"

class VoronoiPathPlanner : public PolygonPathPlanner
{
public:
    VoronoiPathPlanner();

    bool plan(const QPointF &source, const QPointF &terminal, const std::vector<QPolygonF> &polygons, QGraphicsScene *scene, Path &path);

    QString getName() const;
};

#endif // VORONOIPATHPLANNER_H

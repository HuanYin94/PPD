#ifndef VISIBILITYGRAPHPATHPLANNER_H
#define VISIBILITYGRAPHPATHPLANNER_H

#include "polygonpathplanner.h"

class VisibilityGraphPathPlanner : public PolygonPathPlanner
{
public:
    VisibilityGraphPathPlanner();

    bool plan(const QPointF &source, const QPointF &terminal, const std::vector<QPolygonF> &polygons, QGraphicsScene *scene, Path &path);

    QString getName() const;

private:
//    void dijkstra(const std::vector<std::vector<double> > &weight_matrix, int source, int terminal, std::vector<int> &path);
};

#endif // VISIBILITYGRAPHPATHPLANNER_H

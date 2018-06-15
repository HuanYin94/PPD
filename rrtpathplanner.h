#ifndef RRTPATHPLANNER_H
#define RRTPATHPLANNER_H

#include "polygonpathplanner.h"

struct MapState
{
    QPointF point;
    MapState *parent;
    MapState(const QPointF& pot):point(pot), parent(NULL){}
};

class RRTPathPlanner : public PolygonPathPlanner
{
public:

    RRTPathPlanner();

    bool plan(const QPointF &source, const QPointF &terminal, const std::vector<QPolygonF> &polygons, QGraphicsScene *scene, Path &path);

    QString getName() const;

private:
    MapState* getNearestMapState(const std::vector<MapState*>& tree, const QPointF& point);

    bool checkPointValid(const std::vector<QLineF> &polygon_lines, const MapState* state, const QPointF& point);

    bool pointInPolygons(const QPointF &point, const std::vector<QPolygonF> &polygons);

    QPointF randomPoint(const std::vector<QPolygonF> &polygons,
                        double min_x_, double max_x_, double min_y_, double max_y_);

    QPointF generatePointWithinStepInLine(const QPointF& start, const QPointF& end, double step);

    void deleteStatePath();

private:

    std::vector<QLineF> polygon_lines_;
    double min_x_, min_y_, max_x_, max_y_;
    long long circle_nums_;
    double step_;

    std::vector<MapState*> state_path_;
};

#endif // RRTPATHPLANNER_H

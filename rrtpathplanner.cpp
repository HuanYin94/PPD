#include "rrtpathplanner.h"
#include <limits>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <QDebug>

MapState* RRTPathPlanner::getNearestMapState(const std::vector<MapState*>& tree, const QPointF& point)
{
    float min_dist = std::numeric_limits<float>::max();
    MapState *res = NULL;
    for(std::vector<MapState*>::const_iterator i=tree.begin(); i!=tree.end(); i++)
    {
        float cur_dist = distance((*i)->point, point);
        if(cur_dist < min_dist)
        {
            min_dist = cur_dist;
            res = *i;
        }
    }
    return res;
}

bool RRTPathPlanner::checkPointValid(const std::vector<QLineF> &polygon_lines, const MapState* state, const QPointF& point)
{
    QLineF line(state->point, point);
    QPointF tmp;
    for (uint k = 0; k < polygon_lines.size(); ++k)
    {
        if (line.intersect(polygon_lines[k], &tmp) == QLineF::BoundedIntersection)
        {
            return false;
        }
    }
    return true;
}

bool RRTPathPlanner::pointInPolygons(const QPointF &point, const std::vector<QPolygonF> &polygons)
{
    for(size_t i = 0; i!=polygons.size(); i++)
    {
        if(pointInPolygon(point, polygons[i]))
            return true;
    }
    return false;
}

QPointF RRTPathPlanner::randomPoint(const std::vector<QPolygonF> &polygons,
                    double min_x_, double max_x_, double min_y_, double max_y_)
{
    QPointF point;
    do{
        double x = min_x_ + (double)std::rand()/RAND_MAX *(max_x_ - min_x_);
        double y = min_y_ + (double)std::rand()/RAND_MAX *(max_y_ - min_y_);
        point.setX(x); point.setY(y);
    }
    while(pointInPolygons(point, polygons));

    //qDebug() << "random Point : " << point;
    return point;
}

QPointF RRTPathPlanner::generatePointWithinStepInLine(const QPointF& start, const QPointF& end, double step)
{
    if(distance(start, end) <= step)
    {
        //qDebug() << "Finally use this point: " << end;
        return end;
    }
    else
    {
        QLineF line(start, end);
        double angle = line.angle();
        double x = start.x() + step*cos(angle);
        double y = start.y() + step*sin(angle);
        //qDebug() << "Finally use this point: " << QPointF(x,y);
        return QPointF(x,y);
    }
}


RRTPathPlanner::RRTPathPlanner(): circle_nums_(5000), step_(5)
{
    std::srand(static_cast<uint>(time(NULL)));
}


QString RRTPathPlanner::getName() const
{
    return "RRT";
}

bool RRTPathPlanner::plan(const QPointF &source,
                          const QPointF &terminal,
                          const std::vector<QPolygonF> &polygons,
                          QGraphicsScene *scene,
                          Path &path)
{
    // init polygon lines
    polygon_lines_.clear();
    deleteStatePath();

    for (uint pg = 0; pg < polygons.size(); ++pg)
    {
        QPolygonF poly = polygons[pg];
        QPointF p1 = poly[poly.size()-1];
        for (int i = 0; i < poly.size(); ++i)
        {
            QPointF p2 = poly[i];
            polygon_lines_.push_back(QLineF(p1, p2));
            p1 = p2;
        }
    }

    // init map region
    min_x_ = std::numeric_limits<double>::max();
    min_y_ = std::numeric_limits<double>::max();
    max_x_ = -std::numeric_limits<double>::max();
    max_y_ = -std::numeric_limits<double>::max();
    std::vector<QPointF> points;
    points.push_back(source);
    points.push_back(terminal);
    for (uint i = 0; i < polygons.size(); ++i)
    {
        QPolygonF obstacle = polygons[i];
        for (int j = 0; j < obstacle.size(); ++j)
        points.push_back(obstacle[j]);
    }
    for (uint i = 0; i < points.size(); ++i)
    {
        if (min_x_ > points[i].x())
            min_x_ = points[i].x();

        if (min_y_ > points[i].y())
            min_y_ = points[i].y();

        if (max_x_ < points[i].x())
            max_x_ = points[i].x();

        if (max_y_ < points[i].y())
            max_y_ = points[i].y();
    }
    qDebug("min_x = %lf, min_y = %lf, max_x = %lf, max_y = %lf", min_x_, min_y_, max_x_, max_y_);

    // init res tree
    MapState *cur_state = new MapState(source);
    state_path_.push_back(cur_state);

    long long N = circle_nums_;
    while(N--)
    {
        QPointF point = randomPoint(polygons, min_x_, max_x_, min_y_, max_y_);
        MapState* nearest_state = getNearestMapState(state_path_, point);
        point = generatePointWithinStepInLine(nearest_state->point, point, step_);
        if(pointInPolygons(point, polygons) || !checkPointValid(polygon_lines_, nearest_state, point))
            continue;

        cur_state = new MapState(point);
        cur_state->parent = nearest_state;
        state_path_.push_back(cur_state);
        scene->addLine(cur_state->parent->point.x(), cur_state->parent->point.y(), cur_state->point.x(), cur_state->point.y());
        //scene->addEllipse(cur_state->point.x(), cur_state->point.y(), 3, 3);
//        qDebug() << "insert new point " << cur_state->point << " with its parent: " << cur_state->parent->point;

        if(distance(point, terminal) <= step_)
        {
            MapState* final_state = new MapState(terminal);
            final_state->parent = cur_state;
            state_path_.push_back(final_state);

            MapState* state = *state_path_.rbegin();
            std::vector<QPointF> tmp;
            while(state)
            {
                tmp.push_back(state->point);
                state = state->parent;
            }
            qDebug() << "path length: " << tmp.size();
            for(size_t i=tmp.size(); i>0; i--)
            {
                path.push_back(tmp[i-1]);
            }

            deleteStatePath();
            return true;
        }

    }
    deleteStatePath();
    return false;
}

void RRTPathPlanner::deleteStatePath()
{
    for(size_t i = 0; i< state_path_.size(); i++)
    {
        delete state_path_[i];
    }
    state_path_.clear();
}

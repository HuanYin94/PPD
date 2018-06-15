#include "gridpathplanner.h"

#include <limits>
#include <cmath>

GridPathPlanner::GridPathPlanner()
{
}

bool GridPathPlanner::plan(const QPointF &source,
                           const QPointF &terminal,
                           const std::vector<QPolygonF> &polygons,
                           QGraphicsScene *scene,
                           Path &path)
{
    GridMap grid_map;
    int rows, cols;
    PathI pathi;
    QPoint source_new, terminal_new;
    gridize(source, terminal, polygons, source_new, terminal_new, grid_map, rows, cols);

    // Plan
    bool ret = plan(source_new, terminal_new, grid_map, rows, cols, scene, pathi);
    if (ret)
    {
        path.resize(pathi.size());
        for (uint i = 0; i < pathi.size(); ++i)
        {
            path[i].rx() = (pathi[i].rx() + 0.5) * GRID_SIZE + min_x_;
            path[i].ry() = (pathi[i].ry() + 0.5) * GRID_SIZE + min_y_;
        }
    }

    return ret;
}

void GridPathPlanner::gridize(const QPointF &source_in,
        const QPointF &terminal_in,
        const std::vector<QPolygonF> &obstacles,
        QPoint &source_out,
        QPoint &terminal_out,
        GridMap &grid_map, int &rows, int &cols)
{
    min_x_ = std::numeric_limits<double>::max();
    min_y_ = std::numeric_limits<double>::max();
    max_x_ = -std::numeric_limits<double>::max();
    max_y_ = -std::numeric_limits<double>::max();

    std::vector<QPointF> points;
    points.push_back(source_in);
    points.push_back(terminal_in);
    for (uint i = 0; i < obstacles.size(); ++i)
    {
        QPolygonF obstacle = obstacles[i];
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

    min_x_ -= GRID_PADDING;
    min_y_ -= GRID_PADDING;
    max_x_ += GRID_PADDING;
    max_y_ += GRID_PADDING;

    cols = (max_x_ - min_x_) / GRID_SIZE + 0.5;
    rows = (max_y_ - min_y_) / GRID_SIZE + 0.5;

    std::vector<QPolygonF> expanded_obstacles(obstacles.size());
    for (uint i = 0; i < obstacles.size(); ++i)
    {
        QPolygonF obstacle = obstacles[i];

        QPointF center(0.0, 0.0);
        for (int j = 0; j < obstacle.size(); ++j)
            center += obstacle[j];
        center /= obstacle.size();

        QPolygonF expanded_obstacle;
        for (int j = 0; j < obstacle.size(); ++j)
        {
            QPointF v =  obstacle[j] - center;
            v /= std::sqrt(v.x() * v.x() + v.y() * v.y());
            expanded_obstacle.push_back(obstacle[j] + v * GRID_SIZE);
        }

        expanded_obstacles[i] = expanded_obstacle;
    }

    grid_map.resize(rows, std::vector<bool>(cols));
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            QPointF p(j * GRID_SIZE + min_x_, i * GRID_SIZE + min_y_);
            for (uint k = 0; k < obstacles.size(); ++k)
            {
                if (pointInPolygon(p, expanded_obstacles[k]))
                {
                    grid_map[i][j] = true;
                    break;
                }
            }

        }
    }

    source_out = QPoint  ((source_in.x()   - min_x_) / GRID_SIZE + 0.5, (source_in.y()   - min_y_) / GRID_SIZE + 0.5);
    terminal_out = QPoint((terminal_in.x() - min_x_) / GRID_SIZE + 0.5, (terminal_in.y() - min_y_) / GRID_SIZE + 0.5);
}

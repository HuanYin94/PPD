#include "astarpathplanner.h"

#include <queue>
#include <functional>
#include <cmath>
#include <iostream>

#include <QDebug>

struct CostNode
{

public:
    CostNode(int _x, int _y, float _g, float _h) :
        x(_x), y(_y), g(_g), h(_h)
    {
        f = g + h;
    }

    bool operator <(const CostNode &node) const
    {
        return f > node.f;
    }

public:

    int x, y;

    float f, g, h;

    std::vector<QPoint> path;
};

AStarPathPlanner::AStarPathPlanner()
{}

QString AStarPathPlanner::getName() const
{
    return "A*";
}

bool AStarPathPlanner::plan(const QPoint &source,
        const QPoint &terminal,
        const GridMap &grid_map,
        const int rows,
        const int cols,
        QGraphicsScene *scene,
        PathI &path)
{

#if DEBUG
    qDebug() << rows << " row(s), " << cols << " col(s)";
    qDebug() << source;
    qDebug() << terminal;

    for (int j = 0; j < cols; ++j)
    {
        for (int i = 0; i < rows; ++i)
            std::cout << (grid_map[i][j] ? 1 : 0) << " ";
        std::cout << std::endl;
    }
#endif

    int xs = source.x();
    int ys = source.y();
    int xt = terminal.x();
    int yt = terminal.y();

    std::priority_queue<CostNode> cost_nodes;

    std::vector<std::vector<bool> > visited(rows, std::vector<bool>(cols, false));

    CostNode ns(xs, ys, 0, distance(source, terminal));
    ns.path.push_back(source);
    cost_nodes.push(ns);
    visited[ys][xs] = true;

    while (true)
    {
        const CostNode n_best = cost_nodes.top();
        cost_nodes.pop();

        int x = n_best.x;
        int y = n_best.y;

        if (x == xt && y == yt)
        {
            path = n_best.path;
            break;
        }

        // 1 2 3
        // 8   4
        // 7 6 5
        static const double gs[] = {1.414, 1.000, 1.414, 1.000, 1.414, 1.000, 1.414, 1.000};
        static const int    dx[] = {-1,    0,     1,     1,     1,     0,     -1,    -1   };
        static const int    dy[] = {-1     -1,    -1,    0,     1,     1,     1,     0    };

        for (int i = 0; i < 8; ++i)
        {
            int x_new = x + dx[i];
            int y_new = y + dy[i];
            if (
                    x_new >= 0 && y_new >= 0 &&
                    x_new < cols && y_new < rows &&
                    !visited[y_new][x_new] &&
                    !grid_map[y_new][x_new])
            {
                CostNode n_new(x_new, y_new, n_best.g + gs[i], distance(QPoint(x_new, y_new), terminal));
                n_new.path = n_best.path;
                n_new.path.push_back(QPoint(x_new, y_new));
                cost_nodes.push(n_new);
                visited[y_new][x_new] = true;
            }
        }
    }

    return true;
}

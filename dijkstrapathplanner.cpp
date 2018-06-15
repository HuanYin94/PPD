#include "dijkstrapathplanner.h"

#include <QDebug>
#include <iostream>
#include <limits>

DijkstraPathPlanner::DijkstraPathPlanner()
{
}

QString DijkstraPathPlanner::getName() const
{
    return "Dijkstra";
}

bool DijkstraPathPlanner::plan(const QPoint &source,
        const QPoint &terminal,
        const GridMap &grid_map, const int rows, const int cols,
        QGraphicsScene *scene,
        PathI &path)
{
    // 1 2 3
    // 8   4
    // 7 6 5
    static const double gs[] = {1.414, 1.000, 1.414, 1.000, 1.414, 1.000, 1.414, 1.000};
    static const int    dx[] = {-1,    0,     1,     1,     1,     0,     -1,    -1   };
    static const int    dy[] = {-1     -1,    -1,    0,     1,     1,     1,     0    };

    int xs = source.x();
    int ys = source.y();
    int xt = terminal.x();
    int yt = terminal.y();

    std::vector<std::vector<bool> > visited(rows, std::vector<bool>(cols, false));
//    visited[ys][xs] = true;

    std::vector<std::vector<double> > best(rows, std::vector<double>(cols, std::numeric_limits<double>::max()));
    best[ys][xs] = 0;

    // 1   2   3
    // 8  par  4
    // 7   6   5
    std::vector<std::vector<int> > tra(rows, std::vector<int>(cols, 0));

    for (int k = 0; k < rows * cols; ++k)
    {
        double min_dist = std::numeric_limits<double>::max();
        int min_index_x, min_index_y;
        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {
                if (!visited[y][x] && min_dist > best[y][x])
                {
                    min_dist = best[y][x];
                    min_index_x = x;
                    min_index_y = y;
                }
            }
        }

        visited[min_index_y][min_index_x] = true;

        if (min_index_x == xt && min_index_y == yt)
            break;

        for (int i = 0; i < 8; ++i)
        {
            int x_new = min_index_x + dx[i];
            int y_new = min_index_y + dy[i];
            if (
                    x_new >= 0 && y_new >= 0 &&
                    x_new < cols && y_new < rows &&
                    !visited[y_new][x_new] &&
                    !grid_map[y_new][x_new] &&
                    (min_dist + gs[i] < best[y_new][x_new]))
            {
                best[y_new][x_new] = min_dist + gs[i];
                tra[y_new][x_new] = i;
            }
        }
    }

    std::vector<QPoint> tmp_path;
    int x_cur = xt;
    int y_cur = yt;
    while (!(x_cur == xs && y_cur == ys))
    {
        tmp_path.push_back(QPoint(x_cur, y_cur));
        x_cur -= dx[tra[y_cur][x_cur]];
        y_cur -= dy[tra[y_cur][x_cur]];
    }
    tmp_path.push_back(QPoint(x_cur, y_cur));

    int n = tmp_path.size();
    path.resize(n);
    for (int i = 0; i < n; ++i)
        path[n - 1 - i] = tmp_path[i];

    return true;
}

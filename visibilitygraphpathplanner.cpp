#include "visibilitygraphpathplanner.h"

#include <limits>

#include <QLine>

VisibilityGraphPathPlanner::VisibilityGraphPathPlanner()
{
}

bool VisibilityGraphPathPlanner::plan(const QPointF &source, const QPointF &terminal, const std::vector<QPolygonF> &polygons, QGraphicsScene *scene, Path &path)
{
    std::vector<QLineF> polygon_lines;
    int N = 2;

    for (uint pg = 0; pg < polygons.size(); ++pg)
    {
        QPolygonF poly = polygons[pg];
        QPointF p1 = poly[poly.size()-1];
        for (int i = 0; i < poly.size(); ++i)
        {
            QPointF p2 = poly[i];
            polygon_lines.push_back(QLineF(p1, p2));
            p1 = p2;
            N++;
        }
    }

    std::vector<QPointF> vertex(N);
    std::vector<std::vector<double> > weight(N, std::vector<double>(N, -1));
    vertex[0] = source;
    vertex[1] = terminal;
    int first = 2;
    for (uint pg = 0; pg < polygons.size(); ++pg)
    {
        QPolygonF poly = polygons[pg];
//        QPointF p1 = poly[poly.size() - 1];
        int last_i = poly.size() - 1;
        for (int i = 0; i < poly.size(); ++i)
        {
            QPointF p1 = poly[last_i];
            QPointF p2 = poly[i];
            vertex[first + i] = p2;
            weight[first + i][first + last_i] = weight[first + last_i][first + i] = QLineF(p1, p2).length();
            scene->addLine(QLineF(p1, p2));
            p1 = p2;
            last_i = i;
        }

        first += poly.size();
    }

    QPointF tmp;
    for (uint i = 0; i < N; ++i)
    {
        for (uint j = i + 1; j < N; ++j)
        {
            QLineF ll(vertex[i], vertex[j]);
            QLineF nn = ll.unitVector();
            QPointF pp = nn.p2() - nn.p1();
            ll = QLineF(vertex[i] + pp, vertex[j] - pp);
            bool intersect = false;
            for (uint k = 0; k < polygon_lines.size(); ++k)
            {
                if (ll.intersect(polygon_lines[k], &tmp) == QLineF::BoundedIntersection)
                {
                    intersect = true;
                    break;
                }
            }
            for (uint k = 0; k < polygons.size(); ++k)
            {
                if (pointInPolygon(ll.p1(), polygons[k]) || pointInPolygon(ll.p2(), polygons[k]))
                {
                    intersect = true;
                    break;
                }
            }

            if (!intersect)
            {
                weight[j][i] = weight[i][j] = ll.length();
                scene->addLine(QLineF(vertex[i], vertex[j]));
            }
        }
    }

    std::vector<int> pathi;
    if (!dijkstra(weight, 0, 1, pathi))
        return false;

    path.resize(pathi.size());
    for (uint i = 0; i < pathi.size(); ++i)
    {
        path[i] = vertex[pathi[i]];
    }

    return true;
}

QString VisibilityGraphPathPlanner::getName() const
{
    return "Visibility Graph";
}


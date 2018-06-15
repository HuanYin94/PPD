#include "pathplanner.h"

#include <cmath>

PathPlanner::PathPlanner()
{
}

bool PathPlanner::pointInPolygon(const QPointF &point, const QPolygonF &polygon)
{
    int polyCorners = polygon.size();
    int i, j = polyCorners - 1;
    bool oddNodes = false;
    double x = point.x(), y = point.y();

    for (i=0; i < polyCorners; i++)
    {
        if ( ((polygon[i].y() < y && polygon[j].y() >= y) || (polygon[j].y() < y && polygon[i].y() >= y))
             && (polygon[i].x() <= x || polygon[j].x()<=x))
        {
            oddNodes ^= (polygon[i].x()+(y-polygon[i].y())/(polygon[j].y()-polygon[i].y())*(polygon[j].x()-polygon[i].x())<x);
        }

        j = i;
    }

    return oddNodes;
}


double PathPlanner::distance(const QPoint &p1, const QPoint &p2)
{
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(dx * dx + dy * dy);
}

double PathPlanner::distance(const QPointF &p1, const QPointF &p2)
{
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(dx * dx + dy * dy);
}

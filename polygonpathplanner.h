#ifndef POLYGONPATHPLANNER_H
#define POLYGONPATHPLANNER_H

#include "pathplanner.h"

class PolygonPathPlanner : public PathPlanner
{
public:
    PolygonPathPlanner();

protected:
    bool dijkstra(const std::vector<std::vector<double> > &weight_matrix, int source, int terminal, std::vector<int> &path);
};

#endif // POLYGONPATHPLANNER_H

#include "voronoipathplanner.h"

#include <limits>

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "voronoi/vdefs.h"

#ifdef __cplusplus
} // extern "C"
#endif

#include <QDebug>

int sorted, triangulate, plot, debug, nsites, siteidx ;
float xmin, xmax, ymin, ymax ;
Site * sites ;
Freelist sfl ;

Site *
nextone(void)
{
    Site * s ;

    if (siteidx < nsites)
    {
        s = &sites[siteidx++];
        return (s) ;
    }
    else
    {
        return ((Site *)NULL) ;
    }
}

int
scomp(const void * vs1, const void * vs2)
{
    Point * s1 = (Point *)vs1 ;
    Point * s2 = (Point *)vs2 ;

    if (s1->y < s2->y)
    {
        return (-1) ;
    }
    if (s1->y > s2->y)
    {
        return (1) ;
    }
    if (s1->x < s2->x)
    {
        return (-1) ;
    }
    if (s1->x > s2->x)
    {
        return (1) ;
    }
    return (0) ;
}

void
readsites(std::vector<QPointF> points)
{
    int i ;

    nsites = 0 ;
    sites = (Site *) myalloc(points.size() * sizeof(Site));
    for (int p = 0; p < points.size(); ++p)
    {
        sites[nsites].coord.x = points[p].x();
        sites[nsites].coord.y = points[p].y();
        sites[nsites].sitenbr = nsites ;
        sites[nsites++].refcnt = 0 ;
    }

    qsort((void *)sites, nsites, sizeof(Site), scomp) ;
    xmin = sites[0].coord.x ;
    xmax = sites[0].coord.x ;
    for (i = 1 ; i < nsites ; ++i)
    {
        if(sites[i].coord.x < xmin)
        {
            xmin = sites[i].coord.x ;
        }
        if (sites[i].coord.x > xmax)
        {
            xmax = sites[i].coord.x ;
        }
    }
    ymin = sites[0].coord.y ;
    ymax = sites[nsites-1].coord.y ;
}

VoronoiPathPlanner::VoronoiPathPlanner()
{
    debug = 0;
}

bool VoronoiPathPlanner::plan(const QPointF &source, const QPointF &terminal, const std::vector<QPolygonF> &polygons, QGraphicsScene *scene, Path &path)
{
    freeinit(&sfl, sizeof(Site)) ;

    Site *(*next)()  = nextone;

    double
            x_min = std::numeric_limits<double>::max(),
            y_min = std::numeric_limits<double>::max(),
            x_max = -std::numeric_limits<double>::max(),
            y_max = -std::numeric_limits<double>::max();
    std::vector<QPointF> polygon_points;
    for (int pg = 0; pg < polygons.size(); ++pg)
    {
        QPolygonF polygon = polygons[pg];
        for (int p = 0; p < polygon.size(); ++p)
        {
            polygon_points.push_back(polygon[p]);
            if (x_min > polygon[p].x())
                x_min = polygon[p].x();
            if (x_max < polygon[p].x())
                x_max = polygon[p].x();
            if (y_min > polygon[p].y())
                y_min = polygon[p].y();
            if (y_max < polygon[p].y())
                y_max = polygon[p].y();
        }
    }


    std::vector<QLineF> polygon_lines;
    for (uint pg = 0; pg < polygons.size(); ++pg)
    {
        QPolygonF poly = polygons[pg];
        QPointF p1 = poly[poly.size()-1];
        for (int i = 0; i < poly.size(); ++i)
        {
            QPointF p2 = poly[i];
            polygon_lines.push_back(QLineF(p1, p2));
            p1 = p2;
        }
    }

    polygon_points.push_back(QPointF(x_min - VORONOI_PADDING, y_min - VORONOI_PADDING));
    polygon_points.push_back(QPointF(x_max + VORONOI_PADDING, y_min - VORONOI_PADDING));
    polygon_points.push_back(QPointF(x_max + VORONOI_PADDING, y_max + VORONOI_PADDING));
    polygon_points.push_back(QPointF(x_min - VORONOI_PADDING, y_max + VORONOI_PADDING));

    readsites(polygon_points);


    siteidx = 0 ;
    geominit() ;

#define MAX_VERTEXS 1000
#define MAX_EDGES 1000
    double *vertexs_x = new double[MAX_VERTEXS];
    double *vertexs_y = new double[MAX_VERTEXS];
    int    *edges_s   = new int   [MAX_EDGES];
    int    *edges_t   = new int   [MAX_EDGES];
    int vertexs, edges;
    voronoi(next, vertexs_x, vertexs_y, edges_s, edges_t, &vertexs, &edges);

    for (int i = 0; i < vertexs; ++i)
        qDebug() << "Vertex[" << i << "]: (" << vertexs_x[i] << "," << vertexs_y[i] << ")";
    for (int i = 0; i < edges; ++i)
        qDebug() << "Edge[" << i << "]: " << edges_s[i] << "->" << edges_t[i];

    std::vector<QPointF> voronoi_points;
    std::vector<int> voronoi_indecies(vertexs, -1);
    for (int i = 0; i < vertexs; ++i)
    {
        QPointF p(vertexs_x[i], vertexs_y[i]);
        bool in_polygon = false;
        for (int pg = 0; pg < polygons.size(); ++pg)
        {
            if (pointInPolygon(p, polygons[pg]))
            {
                in_polygon = true;
                break;
            }
        }

        if (p.x() < x_min - VORONOI_PADDING || p.x() > x_max + VORONOI_PADDING || p.y() < y_min - VORONOI_PADDING || p.y() > y_max + VORONOI_PADDING)
            in_polygon = true;

        if (!in_polygon)
        {
            voronoi_indecies[i] = voronoi_points.size();
            voronoi_points.push_back(p);
        }
    }

    voronoi_points.push_back(source);
    voronoi_points.push_back(terminal);

    int N = voronoi_points.size();
    std::vector<std::vector<double> > weight_matrix(N, std::vector<double>(N, -1));
    QPointF tmp;
    for (int i = 0; i < edges; ++i)
    {
        if (!(voronoi_indecies[edges_s[i]] < 0 || voronoi_indecies[edges_t[i]] < 0))
        {
            QLineF ll(voronoi_points[voronoi_indecies[edges_s[i]]], voronoi_points[voronoi_indecies[edges_t[i]]]);

            bool intersect = false;
            for (uint k = 0; k < polygon_lines.size(); ++k)
            {
                if (ll.intersect(polygon_lines[k], &tmp) == QLineF::BoundedIntersection)
                {
                    intersect = true;
                    break;
                }
            }

            if (!intersect)
            {
                weight_matrix[voronoi_indecies[edges_s[i]]][voronoi_indecies[edges_t[i]]] =
                        weight_matrix[voronoi_indecies[edges_t[i]]][voronoi_indecies[edges_s[i]]] =
                        distance(voronoi_points[voronoi_indecies[edges_s[i]]], voronoi_points[voronoi_indecies[edges_t[i]]]);
            }

        }
    }

#if 1
    for (int i = 0; i < voronoi_points.size(); ++i)
        scene->addEllipse(voronoi_points[i].x(), voronoi_points[i].y(), 5, 5);
#endif

    for (uint i = 0; i < N - 2; ++i)
    {
        // Connecting the source
        {
            QLineF ll(voronoi_points[N-2], voronoi_points[i]);
            QLineF nn = ll.unitVector();
            QPointF pp = nn.p2() - nn.p1();
            ll = QLineF(voronoi_points[N-2] + pp, voronoi_points[i] - pp);
            bool intersect = false;
            for (uint k = 0; k < polygon_lines.size(); ++k)
            {
                if (ll.intersect(polygon_lines[k], &tmp) == QLineF::BoundedIntersection)
                {
                    intersect = true;
                    break;
                }
            }
            if (!intersect)
                weight_matrix[N-2][i] = weight_matrix[i][N-2] = ll.length();
        }

        // Connecting the terminal
        {
            QLineF ll(voronoi_points[N-1], voronoi_points[i]);
            QLineF nn = ll.unitVector();
            QPointF pp = nn.p2() - nn.p1();
            ll = QLineF(voronoi_points[N-1] + pp, voronoi_points[i] - pp);
            bool intersect = false;
            for (uint k = 0; k < polygon_lines.size(); ++k)
            {
                if (ll.intersect(polygon_lines[k], &tmp) == QLineF::BoundedIntersection)
                {
                    intersect = true;
                    break;
                }
            }
            if (!intersect)
                weight_matrix[N-1][i] = weight_matrix[i][N-1] = ll.length();
        }
    }

    std::vector<int> pathi;
    bool success = dijkstra(weight_matrix, N - 2, N - 1, pathi);

    if (success)
    {
        path.resize(pathi.size());
        for (uint i = 0; i < pathi.size(); ++i)
        {
            path[i] = voronoi_points[pathi[i]];
        }
    }

    delete[] vertexs_x;
    delete[] vertexs_y;
    delete[] edges_s;
    delete[] edges_t;

    return success;
}

QString VoronoiPathPlanner::getName() const
{
    return "Voronoi";
}


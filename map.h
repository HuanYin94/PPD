#ifndef MAP_H
#define MAP_H

#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QPoint>
#include <QPolygon>

#include "pathplanner.h"

class Map : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit Map(QObject *parent = 0);

    void addSource(QPointF _source);

    void addTerminal(QPointF _terminal);

    void addOstacle(QPolygonF _polygon);

    void plan();

    void setPathPlanner(PathPlanner *planner);

private:
    void clear();

private slots:

    void changed(const QList<QRectF> &);

private:

    QGraphicsEllipseItem *source_, *terminal_;

    std::vector<QGraphicsPolygonItem *> obstacles_;

    QGraphicsPathItem *path_;

    PathPlanner *path_planner_;
};

#endif // MAP_H

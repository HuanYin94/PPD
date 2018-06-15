#include "map.h"

#include <QGraphicsSceneDragDropEvent>
#include <QDebug>
#include <QTime>
#include <QMessageBox>

#include "config.h"

Map::Map(QObject *parent) :
    QGraphicsScene(parent), source_(NULL), terminal_(NULL), path_(NULL)
{
    connect(this, SIGNAL(changed(const QList<QRectF> &) ), this, SLOT(changed( const QList<QRectF> &)));
}

void Map::addSource(QPointF _source)
{
    if (source_)
    {
        qCritical("You cannot add source twice");
        return;
    }

    source_ = this->addEllipse(0, 0, VERTEX_RADIUS, VERTEX_RADIUS, QPen(), QBrush(QColor("red"), Qt::SolidPattern));
    source_->setPos(_source.x(), _source.y());
    source_->setFlags(QGraphicsPolygonItem::ItemIsMovable);
}

void Map::addTerminal(QPointF _terminal)
{
    if (terminal_)
    {
        qCritical("You cannot add terminal twice");
        return;
    }

    terminal_ = this->addEllipse(0, 0, VERTEX_RADIUS, VERTEX_RADIUS, QPen(), QBrush(QColor("blue"), Qt::SolidPattern));
    terminal_->setPos(_terminal.x(), _terminal.y());
    terminal_->setFlags(QGraphicsPolygonItem::ItemIsMovable);
}

void Map::addOstacle(QPolygonF _polygon)
{
    QGraphicsPolygonItem * obstacle = this->addPolygon(_polygon, QPen(), QBrush(Qt::SolidPattern));
    obstacle->setFlags(QGraphicsPolygonItem::ItemIsMovable);
    obstacles_.push_back(obstacle);
}

void Map::plan()
{
    qDebug() << "Start to plan";

    clear();

    QPointF source(source_->scenePos() + QPointF(VERTEX_RADIUS / 2.0,  VERTEX_RADIUS / 2.0));
    QPointF terminal(terminal_->scenePos() + QPointF(VERTEX_RADIUS / 2.0,  VERTEX_RADIUS / 2.0));

    std::vector<QPolygonF> obstacles(obstacles_.size());
    for (uint i = 0; i < obstacles_.size(); ++i)
    {
        QPolygonF polygon = obstacles_[i]->polygon();
        for (int j = 0; j < polygon.size(); ++j)
            obstacles[i].push_back(obstacles_[i]->mapToScene(polygon[j]));
    }

    std::vector<QPointF> path;
    QTime timer;
    timer.start();
    bool ret = path_planner_->plan(source, terminal, obstacles, this, path);
    int takes = timer.elapsed();
    if (ret)
    {
#if 0
        path.push_back(QPointF(0, 0));
        path.push_back(QPointF(100, 100));
        path.push_back(QPointF(200, 300));
#endif

        QString info;
        info = "Done. Takes " + QString::number(takes / 1000.0) + " second(s)";
        QMessageBox::information(dynamic_cast<QWidget *>(this->parent()), QString("Path Planner"),info, QMessageBox::Ok);

        if (path.size() == 0)
        {
            qCritical("Empty path");
            return;
        }

        QPainterPath _path;
        _path.moveTo(path[0]);
        for (uint i = 1; i < path.size(); ++i)
        {
            _path.lineTo(path[i]);
        }

        path_ = this->addPath(_path, QPen(QBrush(QColor("green")), 3));
    }
    else
    {
        QMessageBox::warning(dynamic_cast<QWidget *>(this->parent()), QString("Path Planner"), "Failed", QMessageBox::Ok);
        qCritical() << "Failed to plan";
    }
}

void Map::setPathPlanner(PathPlanner *planner)
{
    this->path_planner_ = planner;
}

void Map::clear()
{
    qDebug() << "Clear the map";
#if 0
    if (path_ && path_->scene() == this)
    {
        this->removeItem(path_);
    }
#else
    QList<QGraphicsItem *> items = this->items();
    qDebug() << items.size() << " item(s)";
    for (int i = 0; i < items.size(); ++i)
    {
        bool fixed = false;
        if (items[i] == source_)
        {
            fixed = true;
        }
        else if (items[i] == terminal_)
        {
            fixed = true;
        }
        else
        {
            for (int ob = 0; ob < obstacles_.size(); ++ob)
            {
                if (items[i] == obstacles_[ob])
                {
                    fixed = true;
                    break;
                }
            }
        }

        if (!fixed)
            this->removeItem(items[i]);
    }

#endif
}

void Map::changed(const QList<QRectF> &)
{
    qDebug() << "scene changed";
}

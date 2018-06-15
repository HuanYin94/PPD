#include "pathplanningdemonstrator.h"
#include "ui_pathplanningdemonstrator.h"

#include <QDebug>
#include <QFileDialog>
#include <QtXml>
#include <QFile>

#include "dijkstrapathplanner.h"
#include "astarpathplanner.h"
#include "visibilitygraphpathplanner.h"
#include "voronoipathplanner.h"
#include "rrtpathplanner.h"

PathPlanningDemonstrator::PathPlanningDemonstrator(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PathPlanningDemonstrator)
{
    ui->setupUi(this);

    map_ = new Map;
    ui->graphics_view->setAcceptDrops(true);
    ui->graphics_view->setScene(map_);

    this->addPlanner(new DijkstraPathPlanner);
    this->addPlanner(new AStarPathPlanner);
    this->addPlanner(new VisibilityGraphPathPlanner);
    this->addPlanner(new VoronoiPathPlanner);
    this->addPlanner(new RRTPathPlanner);
}

PathPlanningDemonstrator::~PathPlanningDemonstrator()
{
    delete ui;
}

void PathPlanningDemonstrator::addPlanner(PathPlanner *planner)
{
    planners_.push_back(planner);
    ui->planner_box->addItem(planner->getName());
}

void PathPlanningDemonstrator::on_actionLoad_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, "Open Image", ".", "Map files (*.map)");
    if (filename.isEmpty())
    {
        qWarning("No file chosen");
        return;
    }

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly))
    {
        qCritical() << "Cannot open file:" << filename;
        return;
    }

    QTextStream stream(&file);

    QPoint source, terminal;
    stream >> source.rx() >> source.ry();
    map_->addSource(source);
    stream >> terminal.rx() >> terminal.ry();
    map_->addTerminal(terminal);

    int ostacle_num;
    stream >> ostacle_num;
    for (int i = 0; i < ostacle_num; ++i)
    {
        int vertex_num;
        stream >> vertex_num;
        QPolygonF polygon(vertex_num);
        for (int v = 0; v < vertex_num; v++)
        {
            stream >> polygon[v].rx() >> polygon[v].ry();
        }
        map_->addOstacle(polygon);
    }
}

void PathPlanningDemonstrator::on_plan_utton_clicked()
{
    map_->plan();
}

void PathPlanningDemonstrator::on_planner_box_currentIndexChanged(int index)
{
    Q_ASSERT(index >= 0 && index < planners_.size());

    map_->setPathPlanner(planners_[index]);
}

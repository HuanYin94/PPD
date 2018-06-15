#-------------------------------------------------
#
# Project created by QtCreator 2016-01-23T12:55:55
#
#-------------------------------------------------

QT       += core gui xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PPD
TEMPLATE = app


SOURCES +=\
        pathplanningdemonstrator.cpp \
    map.cpp \
    pathplanner.cpp \
    gridpathplanner.cpp \
    polygonpathplanner.cpp \
    dijkstrapathplanner.cpp \
    astarpathplanner.cpp \
    visibilitygraphpathplanner.cpp \
    voronoipathplanner.cpp \
    voronoi/heap.c \
    voronoi/voronoi.c \
    voronoi/output.c \
    voronoi/geometry.c \
    voronoi/edgelist.c \
    main.cpp \
    voronoi/memory.c \
    rrtpathplanner.cpp

HEADERS  += pathplanningdemonstrator.h \
    map.h \
    config.h \
    pathplanner.h \
    gridpathplanner.h \
    polygonpathplanner.h \
    dijkstrapathplanner.h \
    astarpathplanner.h \
    visibilitygraphpathplanner.h \
    voronoipathplanner.h \
    rrtpathplanner.h

FORMS    += pathplanningdemonstrator.ui

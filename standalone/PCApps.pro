#-------------------------------------------------
#
# Project created by QtCreator 2015-11-18T23:36:53
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PCApps
TEMPLATE = app

include(PCApps-share.pri)

SOURCES += main.cpp \
    mainwindow.cpp \
    Test/Proto/linearsolver.cpp \
    Test/Proto/pointsmoother.cpp \
    Test/Proto/normalsmoother.cpp

HEADERS  += mainwindow.h \
    Test/Proto/linearsolver.h \
    Test/Proto/indexsort.h \
    Test/testbed.h \
    Test/testdescriptor.h \
    Test/testindexsort.h \
    Test/testlinearsolver.h \
    Test/Proto/pointsmoother.h \
    Test/Proto/normalsmoother.h \
    Test/testnormalsmoother.h \
    Test/testnormalvalidity.h \
    Test/testclusterer.h \
    Test/testinobjcluster.h \
    Test/testnoise.h \
    Test/TestReadOBJ.h

FORMS   += mainwindow.ui

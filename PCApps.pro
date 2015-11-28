#-------------------------------------------------
#
# Project created by QtCreator 2015-11-18T23:36:53
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PCApps
TEMPLATE = app
CONFIG += c++11
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

SOURCES += main.cpp\
        mainwindow.cpp \
    PrjCommon/glwidget.cpp \
    PrjCommon/glvertexmanager.cpp

HEADERS  += mainwindow.h \
    PrjCommon/glwidget.h \
    PrjCommon/glvertexmanager.h

FORMS    += mainwindow.ui

RESOURCES += \
    shaders.qrc

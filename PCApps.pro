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

#LIBS += -lopencv

SOURCES += main.cpp \
    mainwindow.cpp \
    IO/glwidget.cpp \
    IO/glvertexmanager.cpp \
    IO/rgbdfilerw.cpp

HEADERS  += mainwindow.h \
    project_common.h \
    IO/glwidget.h \
    IO/glvertexmanager.h \
    IO/rgbdfilerw.h \

FORMS    += mainwindow.ui

RESOURCES += \
    shaders.qrc

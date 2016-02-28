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

#import openmp
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp
LIBS += -fopenmp

# import opencv
INCLUDEPATH += /home/hyukdoo/MyLibs/opencv-3.1.0/include
LIBS += -L/home/hyukdoo/MyLibs/opencv-3.1.0/lib    \
    -lopencv_core   \
    -lopencv_imgproc    \
    -lopencv_highgui    \
    -lopencv_imgcodecs

# import opencl
INCLUDEPATH += /usr/local/cuda-7.5/include
LIBS += -lOpenCL

SOURCES += main.cpp \
    mainwindow.cpp \
    IO/glwidget.cpp \
    IO/glvertexmanager.cpp \
    IO/rgbdfilerw.cpp \
    project_common.cpp \
    PCWork/pcworker.cpp \
    ClWork/clworker.cpp \
    ClWork/clproperty.cpp

HEADERS  += mainwindow.h \
    project_common.h \
    IO/glwidget.h \
    IO/glvertexmanager.h \
    IO/rgbdfilerw.h \
    PCWork/pcworker.h \
    ClWork/clworker.h \
    ClWork/ocl_macros.h \
    ClWork/clproperty.h \
    ClWork/cloperators.h \
    Share/sharedenums.h

FORMS    += mainwindow.ui

RESOURCES += \
    shaders.qrc

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

# import eigen
INCLUDEPATH += /usr/local/include/eigen3

SOURCES += main.cpp \
    mainwindow.cpp \
    Share/project_common.cpp \
    ClUtils/clsetup.cpp \
    ClUtils/cl_utils.cpp \
    IO/glwidget.cpp \
    IO/glvertexmanager.cpp \
    IO/rgbdfilerw.cpp \
    IO/drawutils.cpp \
    IO/VirtualSensor/virtualrgbdsensor.cpp  \
    PCWork/pcworker.cpp \
    PCWork/radiussearch.cpp \
    PCWork/normalmaker.cpp \
    PCWork/descriptormaker.cpp \
    PCWork/descriptormakerbycpu.cpp \
    PCWork/Clustering/planeclusterpolicy.cpp \
    PCWork/Clustering/smallplanemerger.cpp \
    PCWork/Clustering/objectclusterbase.cpp \
    PCWork/Clustering/objectclusterer.cpp \
    PCWork/Clustering/mergeablegraph.cpp \
    PCWork/Clustering/clustererbydbrect.cpp \
    Test/Proto/linearsolver.cpp \
    Test/Proto/descriptormakercpu.cpp \
    Test/Proto/pointsmoother.cpp \
    Test/Proto/normalsmoother.cpp

HEADERS  += mainwindow.h \
    Share/project_common.h \
    Share/forsearchneigbhor.h \
    Share/fordescriptor.h \
    Share/camera_param.h \
    Share/arraydata.h \
    Share/shared_enums.h \
    Share/shared_data.h \
    Share/shared_types.h \
    Share/range.h \
    Share/annotation.h \
    ClUtils/cl_macros.h \
    ClUtils/clsetup.h \
    ClUtils/cloperators.h \
    ClUtils/cl_utils.h \
    ClUtils/clbase.h \
    IO/glwidget.h \
    IO/glvertexmanager.h \
    IO/rgbdfilerw.h \
    IO/imageconverter.h \
    IO/drawutils.h \
    IO/VirtualShape/ivirtualshape.h \
    IO/VirtualShape/virtualrectplane.h \
    IO/VirtualShape/virtualsphere.h \
    IO/VirtualSensor/shapereader.h \
    IO/VirtualSensor/virtualrgbdsensor.h \
    IO/VirtualSensor/posereader.h \
    IO/VirtualSensor/readerutil.h \
    IO/VirtualShape/virtualcuboid.h \
    IO/VirtualShape/virtualellipsoid.h \
    IO/VirtualSensor/attribtype.h \
    IO/VirtualShape/virtualcylinder.h \
    PCWork/pcworker.h \
    PCWork/radiussearch.h \
    PCWork/normalmaker.h \
    PCWork/descriptormaker.h \
    PCWork/descriptormakerbycpu.h \
    PCWork/Clustering/clusterer.h \
    PCWork/Clustering/planeclusterpolicy.h \
    PCWork/Clustering/segment.h \
    PCWork/Clustering/smallplanemerger.h \
    PCWork/Clustering/imline.h \
    PCWork/Clustering/objectclusterbase.h \
    PCWork/Clustering/objectclusterer.h \
    PCWork/Clustering/mergeablegraph.h \
    PCWork/Clustering/clustererbydbrect.h \
    Test/Proto/linearsolver.h \
    Test/Proto/descriptormakercpu.h \
    Test/Proto/indexsort.h \
    Test/testbed.h \
    Test/testdescriptor.h \
    Test/testindexsort.h \
    Test/testlinearsolver.h \
    Test/Proto/pointsmoother.h \
    Test/Proto/normalsmoother.h \
    Test/testnormalsmoother.h \
    Test/VirtualSensor/shapes.h \
    Test/VirtualSensor/virtualdepthsensor.h \
    Test/testnormalvalidity.h \
    Test/testclusterer.h \
    Test/testinobjcluster.h \
    IO/VirtualSensor/noisereader.h \
    IO/VirtualSensor/noisegenerator.h \
    Test/testnoise.h

FORMS    += mainwindow.ui

RESOURCES += \
    shaders.qrc

OTHER_FILES +=

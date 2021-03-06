CONFIG += c++11

DEFINES += PCApps_PATH=\\\"$$PWD\\\"
INCLUDEPATH += $$PWD
include($$PWD/IO/FileReaders/DatasetPaths.pri)

#import openmp
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp
LIBS += -fopenmp

LIBRARY_ROOT=/home/cideep/Library/LibsInstalled

# import opencv
INCLUDEPATH += $$LIBRARY_ROOT/opencv-master/include
LIBS += -L$$LIBRARY_ROOT/opencv-master/lib    \
    -lopencv_core   \
    -lopencv_imgproc    \
    -lopencv_highgui    \
    -lopencv_imgcodecs

# import opencl
INCLUDEPATH += /usr/local/cuda/include
LIBS += -lOpenCL

# import eigen
INCLUDEPATH += $$LIBRARY_ROOT/eigen-3.2.8/include/eigen3

SOURCES += $$PWD/Share/project_common.cpp \
    $$PWD/Share/camera_param.cpp \
    $$PWD/ClUtils/clsetup.cpp \
    $$PWD/ClUtils/cl_utils.cpp \
    $$PWD/IO/glwidget.cpp \
    $$PWD/IO/glvertexmanager.cpp \
    $$PWD/IO/drawutils.cpp \
    $$PWD/IO/VirtualSensor/virtualrgbdsensor.cpp  \
    $$PWD/PCWork/pcworker.cpp \
    $$PWD/PCWork/radiussearch.cpp \
    $$PWD/PCWork/normalmaker.cpp \
    $$PWD/PCWork/Descriptor/descriptormaker.cpp \
    $$PWD/PCWork/Descriptor/descriptormakerbycpu.cpp \
    $$PWD/PCWork/Clustering/planeclusterpolicy.cpp \
    $$PWD/PCWork/Clustering/smallplanemerger.cpp \
    $$PWD/PCWork/Clustering/objectclusterbase.cpp \
    $$PWD/PCWork/Clustering/objectclusterer.cpp \
    $$PWD/PCWork/Clustering/mergeablegraph.cpp \
    $$PWD/PCWork/Clustering/clustererbydbrect.cpp \
    $$PWD/IO/FileReaders/rgbdposereader.cpp \
    $$PWD/IO/FileReaders/readerfactory.cpp \
    $$PWD/IO/FileReaders/rgbdreaderinterface.cpp \
    $$PWD/IO/FileReaders/objectreader.cpp \
    $$PWD/IO/FileReaders/rgbdasyncreader.cpp \
    $$PWD/IO/FileReaders/rgbdsyncreader.cpp \
    ../standalone/IO/FileReaders/samplereader.cpp

HEADERS  += $$PWD/Share/project_common.h \
    $$PWD/Share/fordescriptor.h \
    $$PWD/Share/camera_param.h \
    $$PWD/Share/arraydata.h \
    $$PWD/Share/shared_enums.h \
    $$PWD/Share/shared_data.h \
    $$PWD/Share/shared_types.h \
    $$PWD/Share/range.h \
    $$PWD/Share/annotation.h \
    $$PWD/ClUtils/cl_macros.h \
    $$PWD/ClUtils/clsetup.h \
    $$PWD/ClUtils/cloperators.h \
    $$PWD/ClUtils/cl_utils.h \
    $$PWD/ClUtils/clbase.h \
    $$PWD/IO/glwidget.h \
    $$PWD/IO/glvertexmanager.h \
    $$PWD/IO/imageconverter.h \
    $$PWD/IO/drawutils.h \
    $$PWD/IO/VirtualShape/ivirtualshape.h \
    $$PWD/IO/VirtualShape/virtualrectplane.h \
    $$PWD/IO/VirtualShape/virtualsphere.h \
    $$PWD/IO/VirtualSensor/shapereader.h \
    $$PWD/IO/VirtualSensor/virtualrgbdsensor.h \
    $$PWD/IO/VirtualSensor/posereader.h \
    $$PWD/IO/VirtualSensor/readerutil.h \
    $$PWD/IO/VirtualSensor/noisereader.h \
    $$PWD/IO/VirtualSensor/noisegenerator.h \
    $$PWD/IO/VirtualShape/virtualcuboid.h \
    $$PWD/IO/VirtualShape/virtualellipsoid.h \
    $$PWD/IO/VirtualSensor/attribtype.h \
    $$PWD/IO/VirtualShape/virtualcylinder.h \
    $$PWD/PCWork/pcworker.h \
    $$PWD/PCWork/radiussearch.h \
    $$PWD/PCWork/normalmaker.h \
    $$PWD/PCWork/Descriptor/descriptormaker.h \
    $$PWD/PCWork/Descriptor/descriptormakerbycpu.h \
    $$PWD/PCWork/Clustering/clusterer.h \
    $$PWD/PCWork/Clustering/planeclusterpolicy.h \
    $$PWD/PCWork/Clustering/segment.h \
    $$PWD/PCWork/Clustering/smallplanemerger.h \
    $$PWD/PCWork/Clustering/imline.h \
    $$PWD/PCWork/Clustering/objectclusterbase.h \
    $$PWD/PCWork/Clustering/objectclusterer.h \
    $$PWD/PCWork/Clustering/mergeablegraph.h \
    $$PWD/PCWork/Clustering/clustererbydbrect.h \
    $$PWD/IO/FileReaders/rgbdposereader.h \
    $$PWD/Share/pose6dof.h \
    $$PWD/Share/exceptions.h \
    $$PWD/IO/FileReaders/readerfactory.h \
    $$PWD/IO/FileReaders/rgbdreaderinterface.h \
    $$PWD/IO/FileReaders/objectreader.h \
    $$PWD/IO/FileReaders/rgbdasyncreader.h \
    $$PWD/IO/FileReaders/rgbdsyncreader.h \
    ../standalone/IO/FileReaders/samplereader.h

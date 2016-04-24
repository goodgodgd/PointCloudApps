#ifndef SHAREDDATA_H
#define SHAREDDATA_H

#include <CL/cl.h>
#include <QImage>
#include "Share/project_common.h"

class SharedData
{
public:
    SharedData()
        : pointCloud(nullptr)
        , normalCloud(nullptr)
        , descriptors(nullptr)
        , nullityMap(nullptr)
        , segmentMap(nullptr)
        , dataFilled(false)
    {}

    bool dataFilled;

    void SetPointCloud(cl_float4* srcptr) { pointCloud = srcptr; }
    void SetNormalCloud(cl_float4* srcptr) { normalCloud = srcptr; }
    void SetDescriptors(cl_float4* srcptr) { descriptors = srcptr; }
    void SetNullityMap(cl_uchar* srcptr) { nullityMap = srcptr; }
    void SetSegmentMap(cl_int* srcptr) { segmentMap = srcptr; }
    void SetColorImage(QImage srcimg) { colorImg = srcimg; }

    cl_float4* PointCloud() { return pointCloud; }
    cl_float4* NormalCloud() { return normalCloud; }
    cl_float4* Descriptors() { return descriptors; }
    cl_uchar* NullityMap() { return nullityMap; }
    cl_int* SegmentMap() { return segmentMap; }
    QImage& ColorImage() { return colorImg; }

    const cl_float4* ConstPointCloud() { return pointCloud; }
    const cl_float4* ConstNormalCloud() { return normalCloud; }
    const cl_float4* ConstDescriptors() { return descriptors; }
    const cl_uchar* ConstNullityMap() { return nullityMap; }
    const cl_int* ConstSegmentMap() { return segmentMap; }
    const QImage& ConstColorImage() { return colorImg; }

private:
    cl_float4* pointCloud;
    cl_float4* normalCloud;
    cl_float4* descriptors;
    cl_uchar* nullityMap;
    cl_int* segmentMap;
    QImage colorImg;
};

#endif // SHAREDDATA_H

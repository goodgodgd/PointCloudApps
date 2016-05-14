#ifndef OBJECTCLUSTERBASE_H
#define OBJECTCLUSTERBASE_H

#include <utility>
#include <cassert>
#include <functional>
#include <QImage>
#include <Eigen/Dense>
#include "Share/project_common.h"
#include "Share/camera_param.h"
#include "Share/shared_data.h"
#include "Share/shared_enums.h"
#include "Share/shared_types.h"
#include "Share/arraydata.h"
#include "Share/range.h"
#include "ClUtils/cloperators.h"
#include "IO/imageconverter.h"
#include "Test/testinobjcluster.h"
#include "segment.h"
#include "imline.h"

class ObjectClusterBase
{
public:
    enum Enum
    {
        MERGED_PLANE = -1,
        SMALL_LIMIT = 100
    };

    ObjectClusterBase();

    void ClusterPlanes(SharedData* shdDat);
    const cl_int* GetObjectMap();
    const vecSegment* GetObjects();
    const Segment* GetObjectByID(const int ID);

    // for debug
    Segment* GetPlaneByID(const int ID);
    ArrayData<cl_int> srcPlaneArray;
    cl_int* srcPlaneMap;
    vecSegment srcPlanes;

protected:
    void InitClustering(SharedData* shdDat);
    virtual void MergePlanes() {}
    void ExtractValidSegments(const vecSegment& planes, vecSegment& objects);
    bool DoRectsOverlap(const ImRect& firstRect, const ImRect& secondRect);
    ImRect OverlappingRect(const ImRect& firstRect, const ImRect& secondRect);
    bool ArePlanesConnected(const ImRect& ovlRect, const Segment& firstPlane, const Segment& secondPlane, vecPairOfPixels& connPixels);
    bool ArePixelsConnected(const int firstIdx, const cl_float4& firstNormal, const int secondIdx, const cl_float4& secondNormal);
    void AbsorbPlane(Segment& basePlane, Segment& mergedPlane);

    float HeightFromPlane(const Segment& inputPlane, const Segment& basePlane, const bool bAbs=false);
    std::function<float(float,float)> GetDistUpdater(bool bAbs);

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullityMap;
    vecSegment planes;

    ArrayData<cl_int> objectArray;
    cl_int* objectMap;
    vecSegment objects;
    mapPairOfInts mapIdIndex;
};

#endif // OBJECTCLUSTERBASE_H

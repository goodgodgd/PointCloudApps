#ifndef OBJECTCLUSTERBASE_H
#define OBJECTCLUSTERBASE_H

#include <utility>
#include <cassert>
#include <QImage>
//#include <Eigen/Eigen>
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

    template<typename T>
    T HeightFromPlane(const Segment& inputPlane, const Segment& basePlane)
    {
        const float baseDist = fabsf(clDot(basePlane.center, basePlane.normal));
        T extremeDist = baseDist;
        int yitv = smax((inputPlane.rect.yh - inputPlane.rect.yl)/10, 1);
        int xitv = smax((inputPlane.rect.xh - inputPlane.rect.xl)/10, 1);
        int pxidx;

        for(int y=inputPlane.rect.yl; y<=inputPlane.rect.yh; y+=yitv)
        {
            for(int x=inputPlane.rect.xl; x<=inputPlane.rect.xh; x+=xitv)
            {
                pxidx = IMGIDX(y,x);
                if(nullityMap[pxidx] < NullID::PointNull && objectMap[pxidx]==inputPlane.id)
                    extremeDist = UpdateDist(extremeDist, fabsf(clDot(pointCloud[pxidx], basePlane.normal)));
//                    minDist = smin(minDist, ));
            }
        }
        return baseDist - extremeDist;
    }

    float UpdateDist(float srcValue, float neoValue)
    {
        srcValue = smin(srcValue, neoValue);
    }
    Rangef UpdateDist(Rangef srcValue, float neoValue)
    {
        srcValue.high = smax(srcValue.high, neoValue);
        srcValue.low = smin(srcValue.low, neoValue);
    }



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

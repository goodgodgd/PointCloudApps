#ifndef DRAWUTILS_H
#define DRAWUTILS_H

#include <random>
#include "Share/project_common.h"
#include "Share/forsearchneigbhor.h"
#include "Share/fordescriptor.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/shared_types.h"
#include "IO/glvertexmanager.h"

class DrawUtils
{
public:
    DrawUtils();
    static void DrawPointCloud(int viewOption, SharedData* shdDat);
    static void MarkNeighborsOnImage(QImage& srcimg, QPoint point, cl_int* neighborIndices, cl_int* numNeighbors);
    static void DrawOnlyNeighbors(const QPoint pixel, const cl_float4* pointCloud, const cl_float4* normalCloud
                                  , const cl_int* neighborIndices, const cl_int* numNeighbors, const QImage& colorImg);
    static void MarkPoint3D(SharedData* shdDat, const QPoint pixel, const float normalLength=0.2f);
    static void MarkPoint3D(const cl_float4 point, const cl_float4 normal, QRgb color, const float normalLength=0.2f);
    static const QImage& GetColorMap();

private:
    static void SetColorMapByRgbImage(const QImage& rgbImg);
    static void SetColorMapByDescriptor(const DescType* descriptors);
    static void SetColorMapByCluster(const int* segmentMap);
    static void DrawPointCloudImpl(const cl_float4* pointCloud, const cl_float4* normalCloud);
    static void DrawNormalCloud(const cl_float4* pointCloud, const cl_float4* normalCloud
                                , const int normalInterval = 5, const cl_float4 uniformColor = {0.f,0.f,0.f,0.f});
    static inline void DrawNormal(const cl_float4& point, const cl_float4& normal, const cl_float4& ptcolor, const float length=0.02f);
    static void MarkSegments(const vecSegment* segments, const int minPts=100, const float normalLength=0.15f);
    static inline QRgb GetRandomColor(int index);
    static QImage colorMap;

    friend class MainWindow;
};

#endif // DRAWUTILS_H

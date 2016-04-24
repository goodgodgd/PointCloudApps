#ifndef DRAWUTILS_H
#define DRAWUTILS_H

#include <vector>
#include <random>
#include "Share/project_common.h"
#include "Share/forsearchneigbhor.h"
#include "Share/fordescriptor.h"
#include "Share/sharedenums.h"
#include "IO/glvertexmanager.h"

class DrawUtils
{
public:
    DrawUtils();
    static void DrawPointCloud(cl_float4* pointCloud, cl_float4* normalCloud);
    static void DrawNormalCloud(cl_float4* pointCloud, cl_float4* normalCloud
                                , const int normalInterval = 5, const cl_float4 uniformColor = {0.f,0.f,0.f,0.f});
    static void MarkNeighborsOnImage(QImage& srcimg, QPoint point, cl_int* neighborIndices, cl_int* numNeighbors);
    static void MarkPoint3D(cl_float4 point, cl_float4 normal, QRgb color, const float normalLength = 0.2f);
    static void DrawOnlyNeighbors(QPoint pixel, cl_float4* pointCloud, cl_float4* normalCloud
                                  , cl_int* neighborIndices, cl_int* numNeighbors, QImage& colorImg);

    static void SetColorMapByRgbImage(const QImage& rgbImg);
    static void SetColorMapByDescriptor(const cl_float4* descriptors, const cl_uchar* nullityMap);
    static void SetColorMapByCluster(const int* segmentMap);
    static inline QRgb GetRandomColor(int index);
    static const QImage& GetColorMap();
    static QImage colorMap;

private:
    static inline cl_float4 DescriptorToColor(cl_float4 descriptor);
    static inline void DrawNormal(const cl_float4& point, const cl_float4& normal, const cl_float4& ptcolor, const float length = 0.02f);
};

#endif // DRAWUTILS_H

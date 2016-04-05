#ifndef DRAWUTILS_H
#define DRAWUTILS_H

#include "Share/project_common.h"
#include "Share/forsearchneigbhor.h"
#include "Share/fordescriptor.h"
#include "Share/sharedenums.h"
#include "IO/glvertexmanager.h"

class DrawUtils
{
#define NORMAL_INTERV   5

public:
    DrawUtils();
    static void DrawPointCloud(cl_float4* pointCloud, cl_float4* normalCloud, int viewOption, QImage colorImg, cl_float4* descriptorCloud);
    static void DrawNormalCloud(cl_float4* pointCloud, cl_float4* normalCloud, const cl_float4 ptcolor);
    static void MarkNeighborsOnImage(QImage& srcimg, QPoint point, cl_int* neighborIndices, cl_int* numNeighbors);
    static void MarkPoint3D(cl_float4 point, cl_float4 normal, int viewOption, QRgb color, cl_float4 descriptor
                            , const float normalLength = 0.2f);
    static void DrawOnlyNeighbors(QPoint pixel, cl_float4* pointCloud, cl_float4* normalCloud
                                  , cl_int* neighborIndices, cl_int* numNeighbors
                                  , int viewOption, QImage& colorImg, cl_float4* descriptorCloud);

private:
    static inline cl_float4 ConvertToColor(int viewOption, QRgb rgb, cl_float4& descriptor);
    static inline cl_float4 DescriptorToColor(cl_float4 descriptor);
    static inline void DrawNormal(const cl_float4& point, const cl_float4& normal, const cl_float4& ptcolor, const float length = 0.02f);
};

#endif // DRAWUTILS_H

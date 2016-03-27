#ifndef DRAWUTILS_H
#define DRAWUTILS_H

#include "Share/project_common.h"
#include "Share/forsearchneigbhor.h"
#include "Share/fordescriptor.h"
#include "Share/sharedenums.h"
#include "IO/glvertexmanager.h"

class DrawUtils
{
public:
    DrawUtils();
    static void DrawPointCloud(int viewOption, cl_float4* pointCloud, cl_float4* normalCloud, QImage colorImg, cl_float4* descriptorCloud);
    static void MarkNeighborsOnImage(QImage& srcimg, QPoint point, cl_int* neighborIndices, cl_int* numNeighbors);
    static void MarkPoint3D(int viewOption, cl_float4 point, cl_float4 normal, QRgb color, cl_float4 descriptor);
    static void DrawOnlyNeighbors(QPoint pixel, cl_float4* pointCloud, cl_float4* normalCloud
                                  , cl_int* neighborIndices, cl_int* numNeighbors
                                  , int viewOption, QImage& colorImg, cl_float4* descriptorCloud);

private:
    static inline cl_float4 DescriptorToColor(cl_float4 descriptor);
};

#endif // DRAWUTILS_H

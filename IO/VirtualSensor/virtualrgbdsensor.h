#ifndef VIRTUALRGBDSENSOR_H
#define VIRTUALRGBDSENSOR_H

#include <vector>
#include <QMatrix4x4>
#include "Share/project_common.h"
#include "Share/camera_param.h"
#include "Share/arraydata.h"
#include "IO/VirtualShape/ivirtualshape.h"
#include "shapereader.h"
#include "posereader.h"

class VirtualRgbdSensor
{
public:
    VirtualRgbdSensor();
    void MakeVirtualDepth(const QString& shapefile, const QString& posefile, const QString& noisefile);
    void GrabFrame(QImage& colorImg, QImage& depthImg);

private:
    void UpdateDepthMap(IVirtualShape* shape, const QMatrix4x4& campose);
    inline cl_float4 PixelToRay(const cl_int2& pixel, const QMatrix4x4& campose);
    vecpShape shapes;
    QMatrix4x4 campose;
    ArrayData<cl_float> depthArray;
    cl_float* depthMap;

};

#endif // VIRTUALRGBDSENSOR_H

#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include <QImage>
#include "Share/project_common.h"
#include "Share/camera_param.h"

class ImageConverter
{
public:
    ImageConverter();
    static void ConvertToPointCloud(QImage srcimg, cl_float4* pointCloud);
    static void ConvertToGrayImage(QImage srcimg, QImage& dstimg);
};

#endif // IMAGECONVERTER_H

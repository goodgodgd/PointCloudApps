#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include <QImage>
#include "Share/project_common.h"
#include "Share/camera_param.h"
#include "Share/arraydata.h"

class ImageConverter
{
public:
    ImageConverter();

    static inline cl_float4 PixelToPoint(const int u, const int v, const float depth_m)
    {
        return (cl_float4){depth_m, -(u - CameraParam::cth())/CameraParam::flh()*depth_m, -(v - CameraParam::ctv())/CameraParam::flv()*depth_m, 0.f};
    }

    static inline cl_float2 PointToPixel(const cl_float4& point)
    {
        return (cl_float2){-point.y*CameraParam::flh()/point.x + CameraParam::cth(), -point.z*CameraParam::flv()/point.x + CameraParam::ctv()};;
    }

    static cl_float4* ConvertToPointCloud(QImage depthImg)
    {
        static ArrayData<cl_float4> pcdata(IMAGE_HEIGHT*IMAGE_WIDTH);
        cl_float4* pointCloud = pcdata.GetArrayPtr();
        const cl_float4 point0 = (cl_float4){0,0,0,0};

#pragma omp parallel for
        for(int y=0; y<IMAGE_HEIGHT; y++)
        {
            for(int x=0; x<IMAGE_WIDTH; x++)
            {
                QRgb rgb = depthImg.pixel(x, y);
                uint depth = (uint)(rgb & 0xffff);
                if(depth < CameraParam::RangeBeg_mm() || depth > CameraParam::RangeEnd_mm())
                {
                    pointCloud[IMGIDX(y,x)] = point0;
                    continue;
                }
                float depth_mf = depth / 1000.f;
                pointCloud[IMGIDX(y,x)] = PixelToPoint(x, y, depth_mf);
            }
        }
        return pointCloud;
    }

    static void ConvertToGrayImage(QImage srcimg, QImage& dstimg)
    {
        static QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
        const QRgb black = qRgb(0,0,0);

    #pragma omp parallel for
        for(int y=0; y<IMAGE_HEIGHT; y++)
        {
            for(int x=0; x<IMAGE_WIDTH; x++)
            {
                QRgb rgb = srcimg.pixel(x, y);
                uint depth = (uint)(rgb & 0xffff);

                if(depth < CameraParam::RangeBeg_mm() || depth > CameraParam::RangeEnd_mm())
                {
                    image.setPixel(x, y, black);
                    continue;
                }

                int gray = (int)((float)(depth - CameraParam::RangeBeg_mm()) / (float)(CameraParam::RangeEnd_mm() - CameraParam::RangeBeg_mm()) * 256.f);
                gray = smin(smax(gray, 0), 255);
                rgb = qRgb(gray, gray, gray);
                image.setPixel(x, y, rgb);
            }
        }

        dstimg = image;
    }
};

#endif // IMAGECONVERTER_H

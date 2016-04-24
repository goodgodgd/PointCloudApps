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
    static cl_float4* ConvertToPointCloud(QImage depthImg)
    {
        static ArrayData<cl_float4> pcdata(IMAGE_HEIGHT*IMAGE_WIDTH);
        cl_float4* pointCloud = pcdata.GetArrayPtr();

        const float fc = 300.f;
        const float fr = 300.f;
        const int pc = IMAGE_WIDTH/2;
        const int pr = IMAGE_HEIGHT/2;
        const cl_float4 point0 = (cl_float4){0,0,0,0};

#pragma omp parallel for
        for(int y=0; y<IMAGE_HEIGHT; y++)
        {
            for(int x=0; x<IMAGE_WIDTH; x++)
            {
                QRgb rgb = depthImg.pixel(x, y);
                uint depth = (uint)(rgb & 0xffff);
                if(depth < DEAD_RANGE_MM || depth > DEPTH_RANGE_MM)
                {
                    pointCloud[IMGIDX(y,x)] = point0;
                    continue;
                }
                float depth_mf = depth / 1000.f;

                pointCloud[IMGIDX(y,x)].x = depth_mf;
                pointCloud[IMGIDX(y,x)].y = -(x - pc)/fc*depth_mf;
                pointCloud[IMGIDX(y,x)].z = -(y - pr)/fr*depth_mf;
                pointCloud[IMGIDX(y,x)].w = 0.f;
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

                if(depth < DEAD_RANGE_MM || depth > DEPTH_RANGE_MM)
                {
                    image.setPixel(x, y, black);
                    continue;
                }

                int gray = (int)((float)(depth - DEAD_RANGE_MM) / (float)(DEPTH_RANGE_MM - DEAD_RANGE_MM) * 256.f);
                gray = smin(smax(gray, 0), 255);
                rgb = qRgb(gray, gray, gray);
                image.setPixel(x, y, rgb);
            }
        }

        dstimg = image;
    }
};

#endif // IMAGECONVERTER_H

#include "virtualrgbdsensor.h"

VirtualRgbdSensor::VirtualRgbdSensor()
{
    depthArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    depthMap = depthArray.GetArrayPtr();
}

void VirtualRgbdSensor::MakeVirtualDepth(const QString& shapefile, const QString& posefile, const QString& noisefile)
{
    try
    {
        shapes = ShapeReader::ReadShapes(shapefile);
        campose = PoseReader::ReadPose(posefile);
        memset(depthMap, 0x00, depthArray.ByteSize());
        for(auto shape : shapes)
            UpdateDepthMap(shape, campose);
    }
    catch (QString errmsg)
    {
        qDebug() << errmsg;
        return;
    }
//    GaussianParam noiseParam = NoiseParamReader::ReadParams(noisefile);
//    AddDepthNoise(depthMap);
}

void VirtualRgbdSensor::GrabFrame(QImage& colorImg, QImage& depthImg)
{
    static QImage colorFrame(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static QImage depthFrame(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);

    const QRgb black = qRgb(0,0,0);
    const QRgb white = qRgb(255,255,255);
    uint depth_mm;
    QRgb rgb;

//#pragma omp parallel for
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            colorFrame.setPixel(x, y, white);

            assert(depthMap[IMGIDX(y,x)]>=0.f);
            depth_mm = (uint)(depthMap[IMGIDX(y,x)]*1000.f);
            if(depth_mm < DEAD_RANGE_MM || depth_mm > DEPTH_RANGE_MM)
                depthFrame.setPixel(x, y, black);
            else
            {
                rgb = qRgb(0, (depth_mm>>8 & 0xff), (depth_mm & 0xff));
                depthFrame.setPixel(x, y, rgb);
            }
        }
    }

    depthImg = depthFrame;
    colorImg = colorFrame;
}

void VirtualRgbdSensor::UpdateDepthMap(const IVirtualShape* shape, const QMatrix4x4& campose)
{
//    shape->IntersectingPointFromRay()

    const cl_float4 position = (cl_float4){campose(0,3), campose(1,3), campose(2,3), 0};
    const QVector3D tmpdir = campose.mapVector(QVector3D(1,0,0)).normalized();
    const cl_float4 frontdir = (cl_float4){tmpdir.x(), tmpdir.y(), tmpdir.z(), 0};
    cl_float4 raydir;
    cl_float4 intersect;
    float depth;
    const float depth_min_range = DEAD_RANGE_MM/1000.f;

    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            // raydir must have unit length
            raydir = PixelToRay((cl_int2){x,y}, campose);

            if(shape->IntersectingPointFromRay(position, raydir, intersect))
            {
//                if(shape->type==IVirtualShape::RECT && x%5==0 && y%5==0)
//                    qDebug() << "intersect" << intersect << raydir << x << y;

                depth = clDot(frontdir, intersect - position);
                assert(depth>=0);
                if(depthMap[IMGIDX(y,x)] < depth_min_range || depth < depthMap[IMGIDX(y,x)])
                    depthMap[IMGIDX(y,x)] = depth;
            }
        }
    }
}

cl_float4 VirtualRgbdSensor::PixelToRay(const cl_int2& pixel, const QMatrix4x4& campose)
{
    static const int pc = IMAGE_WIDTH/2;
    static const int pr = IMAGE_HEIGHT/2;
    QVector3D rayInCamera(1.f, -(pixel.x - pc)/FOCAL_LENGTH, -(pixel.y - pr)/FOCAL_LENGTH);
    QVector3D rayInGlobal = campose.mapVector(rayInCamera);
    cl_float4 raydir;
    if(rayInGlobal.length() < 0.01f)
        qDebug() << "ray in global" << rayInGlobal << rayInCamera;
    raydir << rayInGlobal;
    return clNormalize(raydir);
}

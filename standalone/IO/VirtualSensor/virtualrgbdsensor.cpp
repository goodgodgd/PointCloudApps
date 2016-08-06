#include "virtualrgbdsensor.h"

VirtualRgbdSensor::VirtualRgbdSensor()
    : randGen(nullptr)
{
    depthArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    depthMap = depthArray.GetArrayPtr();
}

void VirtualRgbdSensor::MakeVirtualDepth(const QString& shapefile, const QString& posefile, const QString& noisefile)
{
    try
    {
        memset(depthMap, 0x00, depthArray.ByteSize());
        shapes = ShapeReader::ReadShapes(shapefile);
        campose = PoseReader::ReadPose(posefile);
        for(auto shape : shapes)
            UpdateDepthMap(shape, campose);

        if(randGen!=nullptr)
            delete randGen;
        randGen = NoiseReader::ReadNoiseGenerator(noisefile);
        AddNoiseToDepth(randGen);
    }
    catch (QString errmsg)
    {
        qDebug() << errmsg;
        return;
    }
}

void VirtualRgbdSensor::GrabFrame(QImage& colorImg, QImage& depthImg)
{
    static QImage colorFrame(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static QImage depthFrame(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);

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
            rgb = qRgb(0, (depth_mm>>8 & 0xff), (depth_mm & 0xff));
            depthFrame.setPixel(x, y, rgb);
        }
    }

    depthImg = depthFrame;
    colorImg = colorFrame;
}

void VirtualRgbdSensor::UpdateDepthMap(IVirtualShape* shape, const QMatrix4x4& campose)
{
    const cl_float4 position = (cl_float4){campose(0,3), campose(1,3), campose(2,3), 0};
    QMatrix4x4 glob2camera = campose.inverted();
    cl_float4 raydir, intersect;
    QVector3D intersectInGlobal, intersectInCamera;
    float depth;
    const float depth_min_range = CameraParam::RangeBeg_m();

    if(shape->type==IVirtualShape::CUBOID)
    {
        VirtualCuboid* cuboid = static_cast<VirtualCuboid*>(shape);
        cuboid->outlierCount = cuboid->totalCount = 0;
    }

    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            // raydir must have unit length
            raydir = PixelToRay((cl_int2){x,y}, campose);

            if(shape->IntersectingPointFromRay(position, raydir, intersect))
            {
                intersectInGlobal << intersect;
                intersectInCamera = glob2camera.map(intersectInGlobal);
                depth = intersectInCamera.x();
                assert(depth>=0);
                if(depthMap[IMGIDX(y,x)] < depth_min_range || depth < depthMap[IMGIDX(y,x)])
                    depthMap[IMGIDX(y,x)] = depth;
            }
        }
    }

    if(shape->type==IVirtualShape::CUBOID)
    {
        VirtualCuboid* cuboid = static_cast<VirtualCuboid*>(shape);
        qDebug() << "outliers" << cuboid->outlierCount << cuboid->totalCount;
    }
}

cl_float4 VirtualRgbdSensor::PixelToRay(const cl_int2& pixel, const QMatrix4x4& campose)
{
    QVector3D rayInCamera(1.f, -(pixel.x - CameraParam::cth())/CameraParam::flh(), -(pixel.y - CameraParam::ctv())/CameraParam::flv());
    QVector3D rayInGlobal = campose.mapVector(rayInCamera);
    cl_float4 raydir;
    if(rayInGlobal.length() < 0.01f)
        qDebug() << "ray in global" << rayInGlobal << rayInCamera;
    raydir << rayInGlobal;
    return clNormalize(raydir);
}

void VirtualRgbdSensor::AddNoiseToDepth(RandGenerator* randGen)
{
    const float depth_min_range = CameraParam::RangeBeg_m();
    int pxidx;
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            pxidx = IMGIDX(y,x);
            if(depthMap[pxidx] > depth_min_range)
                depthMap[pxidx] += depthMap[pxidx]*depthMap[pxidx]*randGen->Generate();
        }
    }
}

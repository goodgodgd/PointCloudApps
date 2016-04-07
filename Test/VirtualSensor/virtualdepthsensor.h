#ifndef VIRTUALDEPTHSENSOR_H
#define VIRTUALDEPTHSENSOR_H

#include <QMatrix4x4>
#include <QImage>
#include "Share/project_common.h"
#include "shapes.h"

namespace Shape
{
enum eShape
{
    RECT,
    CYLINDER,
    CIRCLE,
    CUBE        // apppend this!!
};
}

class ShapeReader
{
public:
    ShapeReader() {}
    const vector<IShape*>& ReadShapes(const char* filename)
    {
        // use QFile instead
        FILE* fp = fopen(filename, "r");

        shapes.clear();
        while(!feof(fp))
        {
            int type = ReadType(fp);
            if(type==Shape::RECT)
                shapes.push_back(new Rect(fp));
            if(type==Shape::CYLINDER)
                shapes.push_back(new Cylinder(fp));
            if(type==Shape::CIRCLE)
                shapes.push_back(new Circle(fp));
        }
        return shapes;
    }

private:
    Shape::eShape ReadType(FILE* fp) {}
    vector<IShape*> shapes;
};

class PoseReader
{
public:
    PoseReader() {}
    static QMatrix4x4 ReadPose(const char* filename) {}
};

struct GaussianParam
{
    float mu;
    float sigma;
};

class NoiseParamReader
{
public:
    NoiseParamReader() {}
    static GaussianParam ReadParams(const char* filename) {}
};

class VirtualDepthSensor
{
public:
    VirtualDepthSensor() {}
    void MakeVirtualDepth(const char* shapefile, const char* posefile, const char* noisefile)
    {
        ShapeReader shapeReader;
        const vector<IShape*>& shapes = shapeReader.ReadShapes(shapefile);
        QMatrix4x4 campose = PoseReader::ReadPose(posefile);
        for(auto shape : shapes)
            UpdateDepthMap(shape, campose, depthMap);

        GaussianParam noiseParam = NoiseParamReader::ReadParams(noisefile);
        AddDepthNoise(depthMap);
    }
    QImage GetDepthFrame()
    {
        return depthImg;
    }

private:
    void UpdateDepthMap(IShape* shape, const QMatrix4x4& campose, float* depthMap)
    {
        const cl_float4 position = GetPosition(campose);
        const cl_float4 frontdir = GetFrontDir(campose);
        cl_float4 raydir;
        cl_float4 intersect;
        float depth;
        for(int y=0; y<IMAGE_HEIGHT; y++)
        {
            for(int x=0; x<IMAGE_WIDTH; x++)
            {
                // raydir must be scaled for x to be 1
                raydir = PixelToRay((cl_int2){x,y});
                if(shape->DoesRayIntersect(position, raydir))
                {
                    intersect = shape->IntersectingPoint(position, raydir);
                    depth =  clDot(frontdir, intersect - position);
                    if(depth < depthMap[IMGIDX(y,x)])
                        depthMap[IMGIDX(y,x)] = depth;
                }
            }
        }
    }

    void AddDepthNoise(float* depthMap) {}
    cl_float4 GetPosition(const QMatrix4x4& campose) {}
    cl_float4 GetFrontDir(const QMatrix4x4& campose) {}
    cl_float4 PixelToRay(const cl_int2& pixel) {}

    float depthMap[IMAGE_WIDTH*IMAGE_HEIGHT];
    QImage depthImg;
};

#endif // VIRTUALDEPTHSENSOR_H

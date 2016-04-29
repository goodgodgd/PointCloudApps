#ifndef VIRTUALDEPTHSENSOR_H
#define VIRTUALDEPTHSENSOR_H

#include <QMatrix4x4>
#include <QVector3D>
#include <QImage>
#include <QFile>
#include <QString>
#include <QStringList>
#include "Share/project_common.h"
#include "shapes.h"
#include "Share/camera_param.h"

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

#define FILEPATH  "/home/seongwon/Work/PointCloudApps/Test/VirtualSensor/"
class ShapeReader
{
public:
    ShapeReader() {}
    const vector<IShape*>& ReadShapes(const char* filename)
    {
        QString name = QString(FILEPATH) + QString(filename);
        QFile* fp = new QFile;
        fp->setFileName(name);
        fp->open(QIODevice::ReadOnly);
        QTextStream* read = new QTextStream(fp);
        shapes.clear();
        while(!read->atEnd())
        {
            int type = ReadType(read);
            if(type==Shape::RECT)
                shapes.push_back(new Rect(read));
            else if(type==Shape::CYLINDER)
                shapes.push_back(new Cylinder(read));
            else if(type==Shape::CIRCLE)
                shapes.push_back(new Circle(read));
            else if(type==Shape::CUBE)
                shapes.push_back(new Cube(read));
        }
        return shapes;
    }

private:
    Shape::eShape ReadType(QTextStream* read) {
        QString tmp = read->readLine();
        QStringList tmpList = tmp.split("=");
        if(tmpList[1]=="rect")
            return Shape::RECT;
        else if(tmpList[1]=="cylinder")
            return Shape::CYLINDER;
        else if(tmpList[1]=="circle")
            return Shape::CIRCLE;
        else if(tmpList[1]=="cube")
            return Shape::CUBE;
    }
    vector<IShape*> shapes;
};

class PoseReader
{
public:
    PoseReader() {}
    static QMatrix4x4 ReadPose(const char* filename) {

       float pos_val[3], rot_val[3];

        QString name = QString(FILEPATH) + QString(filename);
        QFile* fp = new QFile;
        fp->setFileName(name);
        fp->open(QIODevice::ReadOnly);
        QTextStream* read = new QTextStream(fp);

        for(int i=0;i<3;i++){
            QString tmp = read->readLine();
            QStringList tmpList = tmp.split("=");
            pos_val[i] = tmpList[1].toFloat();
        }
        for(int i=0;i<3;i++){
            QString tmp = read->readLine();
            QStringList tmpList = tmp.split("=");
            rot_val[i] = tmpList[1].toFloat();
        }

        /*QVector3D cameraPosition = QVector3D::QVector3D(pos_val[0],pos_val[1],pos_val[2]);

        QMatrix4x4 cameraTransformation;
        cameraTransformation.rotate(rot_val[0], 1,0,0);
        cameraTransformation.rotate(rot_val[1], 0,1,0);
        cameraTransformation.rotate(rot_val[2], 1,0,1);

        QVector3D cameraUpdirection = cameraTransformation * QVector3D(0,0,1);
        QMatrix4x4 viewMatrix;
        viewMatrix.lookAt(cameraPosition, QVector3D(1,0,0), cameraUpdirection-cameraPosition);
        return viewMatrix;*/

        QMatrix4x4 viewPose;
        viewPose.setToIdentity();

        QVector3D transVec;

        //x rot
        for(int i=0;i<3;i++){
            float rotDegree = rot_val[i];
            viewPose.rotate(rotDegree, int(i==0),int(i==1),int(i==2));
        }
        viewPose.translate(pos_val[0],pos_val[1],pos_val[2]);

        QMatrix4x4 viewMatrix;
        QVector3D localPoint;
        localPoint = QVector3D(0,0,0);
        QVector3D eye = viewPose.map(localPoint);
        localPoint = QVector3D(1,0,0);
        QVector3D center = viewPose.map(localPoint);
        localPoint = QVector3D(0,0,1);
        QVector3D up = viewPose.map(localPoint) - eye;
        viewMatrix.setToIdentity();
        viewMatrix.lookAt(eye,center,up);

        return viewMatrix;

    }
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
    static GaussianParam ReadParams(const char* filename) {
        GaussianParam parameter;
        QString name = QString(FILEPATH) + QString(filename);
        QFile* fp = new QFile;
        fp->setFileName(name);
        fp->open(QIODevice::ReadOnly);
        QTextStream* read = new QTextStream(fp);
        QString tmp = read->readLine();
        QStringList tmpList = tmp.split("=");
        parameter.mu = tmpList[1].toFloat();
        tmp = read->readLine();
        tmpList = tmp.split("=");
        parameter.sigma = tmpList[1].toFloat();
        return parameter;
    }
};

class VirtualDepthSensor
{
public:
    VirtualDepthSensor() {
       // projection.setToIdentity();
       // projection.perspective(45, 4.f/3.f, 0.1f, 100.f);
    }
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
                raydir = PixelToRay((cl_int2){x,y}, campose);
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
    cl_float4 GetPosition(const QMatrix4x4& campose) {
        cl_float4 position;
        position.x = campose.column(3).x();
        position.y = campose.column(3).y();
        position.z = campose.column(3).z();
        position.w = 0.f;
        return position;
    }
    cl_float4 GetFrontDir(const QMatrix4x4& campose) {
        // http://db-in.com/images/local_vectors.jpg
        cl_float4 direction;
        direction.x = campose.row(2).x();
        direction.y = campose.row(2).y();
        direction.z = campose.row(2).z();
        direction.z = 0.f;
        return direction;
    }
    cl_float4 PixelToRay(const cl_int2& pixel, const QMatrix4x4& campose) {
        float fov = 45*PI_F/180.f;
        float ratio = IMAGE_WIDTH/(float)IMAGE_HEIGHT;
        float halfWidth = IMAGE_WIDTH * 0.5f;
        float halfHeight = IMAGE_HEIGHT * 0.5f;

        float dx = std::tan(fov * 0.5f) * (pixel.x / halfWidth - 1.0f) / ratio;
        float dy = std::tan(fov * 0.5f) * (pixel.y / halfHeight - 1.0f);

        QVector4D point(1.f,-dy,-dx,0.f);
        QMatrix4x4 invview = campose.inverted();
        point = invview.map(point);

        cl_float4 ray;
        ray.x = point.x();
        ray.y = point.y();
        ray.z = point.z();
        ray.w = point.w();

        return ray;

    }

    struct camera{
        cl_float4 position;
        cl_float4 direction;
        QVector3D eye;
        QVector3D center;
        QVector3D up;
    };

   // QMatrix4x4 projection;



    float depthMap[IMAGE_WIDTH*IMAGE_HEIGHT];
    QImage depthImg;
};

#endif // VIRTUALDEPTHSENSOR_H

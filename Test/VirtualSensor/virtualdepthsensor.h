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
    SPHERE,
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
            else if(type==Shape::SPHERE)
                shapes.push_back(new Sphere(read));
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
        else if(tmpList[1]=="sphere")
            return Shape::SPHERE;
        else if(tmpList[1]=="cube")
            return Shape::CUBE;
    }
    vector<IShape*> shapes;
};

class PoseReader
{
public:
    PoseReader() {}
    static void ReadPose(const char* filename, QMatrix4x4& viewPose, QVector3D& position, QVector3D& frontdir, QVector3D& upvector) {

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

        viewPose.setToIdentity();
        viewPose.translate(pos_val[0],pos_val[1],pos_val[2]);
        for(int i=0;i<3;i++){
            viewPose.rotate(rot_val[i], int(i==0),int(i==1),int(i==2));
        }

        QVector3D localPoint;
        localPoint = QVector3D(0,0,0);
        QVector3D eye = viewPose.map(localPoint);
        localPoint = QVector3D(1,0,0);
        QVector3D center = viewPose.map(localPoint);
        localPoint = QVector3D(0,0,1);
        QVector3D up = viewPose.map(localPoint) - eye;

        position = eye;
        frontdir = (center-eye).normalized();
        upvector = up.normalized();

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
    QVector3D position3, frontdir3, up3;
    VirtualDepthSensor() {
        for(int i=0;i<IMAGE_WIDTH*IMAGE_HEIGHT;i++){
            depthMap[i]=DEPTH_RANGE_MM+1;
        }
    }
    void MakeVirtualDepth(const char* shapefile, const char* posefile, const char* noisefile)
    {
        ShapeReader shapeReader;
        const vector<IShape*>& shapes = shapeReader.ReadShapes(shapefile);
        QMatrix4x4 campose;
        PoseReader::ReadPose(posefile, campose, position3, frontdir3, up3);
        for(auto shape : shapes)
            UpdateDepthMap(shape, campose);

        GaussianParam noiseParam = NoiseParamReader::ReadParams(noisefile);
        AddDepthNoise(depthMap);
    }
    QImage GetDepthFrame()
    {

        static QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
        static QImage image_test(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
        const QRgb black = qRgb(0,0,0);

    #pragma omp parallel for
        for(int y=0; y<IMAGE_HEIGHT; y++)
        {
            for(int x=0; x<IMAGE_WIDTH; x++)
            {

                uint depth = depthMap[IMGIDX(y,x)];
                if(depth == 3501){
                    image_test.setPixel(x,y,black);
                }
                else{
                    QRgb red = qRgb(255,0,0);
                    image_test.setPixel(x,y, red);
                }

                if(depth < DEAD_RANGE_MM || depth > DEPTH_RANGE_MM)
                {
                    image.setPixel(x, y, black);
                }
                else{
                    int gray = (int)((float)(depth - DEAD_RANGE_MM) / (float)(DEPTH_RANGE_MM - DEAD_RANGE_MM) * 256.f);
                    gray = smin(smax(gray, 0), 255);
                    QRgb rgb = qRgb(gray, gray, gray);
                    image.setPixel(x, y, rgb);

                }
            }
        }

            depthImg = image;
        depthImg.save("/home/seongwon/Work/PointCloudApps/Test/VirtualSensor/gray.png");

        //image_test is just looking where ray intersect occurs.
        image_test.save("/home/seongwon/Work/PointCloudApps/Test/VirtualSensor/test.png");

        return depthImg;
    }

private:
    void UpdateDepthMap(IShape* shape, const QMatrix4x4& campose)
    {
        const cl_float4 position = GetPosition(position3);
        const cl_float4 frontdir = GetFrontDir(frontdir3);
        cl_float4 raydir;
        cl_float4 intersect;
        float depth;
        qDebug() << position << frontdir;
        for(int y=0; y<IMAGE_HEIGHT; y++)
        {
            for(int x=0; x<IMAGE_WIDTH; x++)
            {
                // raydir must be scaled for x to be 1
                raydir = PixelToRay((cl_int2){x,y}, campose, position3);
                if(shape->DoesRayIntersect(position, raydir))
                {
                    intersect = shape->IntersectingPoint(position, raydir);
                    depth =  clDot(frontdir, intersect - position);

                    if(depth < depthMap[IMGIDX(y,x)]){
                        depthMap[IMGIDX(y,x)] = depth*100; //scaling depth
                        qDebug() << depth << depthMap[IMGIDX(y,x)] << x << y;
                    }
                }
            }
        }
    }
    void AddDepthNoise(float* depthMap) {}
    cl_float4 GetPosition(QVector3D& position3) {
        cl_float4 position;
        position.x = position3.x();
        position.y = position3.y();
        position.z = position3.z();
        position.w = 0.f;
        return position;
    }
    cl_float4 GetFrontDir(QVector3D& frontdir3) {
        cl_float4 direction;
        direction.x = frontdir3.x();
        direction.y = frontdir3.y();
        direction.z = frontdir3.z();
        direction.w = 0.f;
        return direction;
    }
    cl_float4 PixelToRay(const cl_int2& pixel, const QMatrix4x4& campose, QVector3D position) {
        float fov = 45*PI_F/180.f;
        float ratio = IMAGE_WIDTH/(float)IMAGE_HEIGHT;

        float halfWidth = IMAGE_WIDTH * 0.5f;
        float halfHeight = IMAGE_HEIGHT * 0.5f;

        float dx = std::tan(fov * 0.5f) * ((pixel.x + 0.5) / halfWidth - 1.0f);
        float dy = std::tan(fov * 0.5f) * ((pixel.y+ 0.5) / halfHeight - 1.0f) / ratio;


        QVector3D point(1.f, -dx, -dy);

        QMatrix4x4 invview;
        invview = campose.inverted();
        point = invview.map(point);
        QVector3D zero(0.f,0.f,0.f);
        zero = invview.map(zero);
        point = point - zero;

        cl_float4 ray;
        ray.x = point.x();
        ray.y = point.y();
        ray.z = point.z();
        ray.w = 0.f;
        ray = clNormalize(ray);

        return ray;
    }


    float depthMap[IMAGE_WIDTH*IMAGE_HEIGHT];
    QImage depthImg;
};

#endif // VIRTUALDEPTHSENSOR_H

#ifndef SHAPES_H
#define SHAPES_H

#include "Share/project_common.h"
#include <vector>
#include <QFile>
#include <QString>
#include <QStringList>


struct IShape
{
    // virtual descturctor must be defined in interface class
    virtual ~IShape() {}
    // =0; means derived classes must implement the function
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) = 0;
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) = 0;
protected:
    cl_float4 center;
};

class Rect : public IShape
{
public:
    Rect(QTextStream* read) {
        float data[5];
        for(int i=0;i<5;i++){
            QString tmp = read.readLine();
            QStringList tmpList = tmp.split("=");
            data[i] = tmpList[1].toFloat();
        }
        center.x = data[0];
        center.y = data[1];
        center.z = data[2];
        width = data[3];
        height = data[4];
        read.readLine();
    }
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {}
private:
    float width;
    float height;
};

class Cylinder : public IShape
{
public:
    Cylinder(QTextStream* read) {
        float data[5];
        for(int i=0;i<5;i++){
            QString tmp = read.readLine();
            QStringList tmpList = tmp.split("=");
            data[i] = tmpList[1].toFloat();
        }
        center.x = data[0];
        center.y = data[1];
        center.z = data[2];
        radius = data[3];
        height = data[4];
        read.readLine();
    }
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {}
private:
    float radius;
    float height;
};

class Circle : public IShape
{
public:
    Circle(QTextStream* read) {
        float data[4];
        for(int i=0;i<4;i++){
            QString tmp = read.readLine();
            QStringList tmpList = tmp.split("=");
            data[i] = tmpList[1].toFloat();
        }
        center.x = data[0];
        center.y = data[1];
        center.z = data[2];
        radius = data[3];
        read.readLine();
    }
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {}
private:
    float radius;
};

class Cube : public IShape
{
public:
    Cube(QTextStream* read) {
        float data[6];
        for(int i=0;i<6;i++){
            QString tmp = read.readLine();
            QStringList tmpList = tmp.split("=");
            data[i] = tmpList[1].toFloat();
        }
        center.x = data[0];
        center.y = data[1];
        center.z = data[2];
        width = data[3];
        height = data[4];
        depth = data[5];
        read.readLine();
    }
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {}
private:
    float width;
    float height;
    float depth;
};

#endif // SHAPES_H

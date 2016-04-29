#ifndef SHAPES_H
#define SHAPES_H

#include "Share/project_common.h"
#include "ClUtils/cloperators.h"
#include <vector>
#include <QFile>
#include <QString>
#include <QStringList>


#define MAX2(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a > _b ? _a : _b; })

#define MIN2(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a < _b ? _a : _b; })

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
            QString tmp = read->readLine();
            QStringList tmpList = tmp.split("=");
            data[i] = tmpList[1].toFloat();
        }
        center.x = data[0];
        center.y = data[1];
        center.z = data[2];
		center.w = 0.f;
        width = data[3];
        height = data[4];
        read->readLine();
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
            QString tmp = read->readLine();
            QStringList tmpList = tmp.split("=");
            data[i] = tmpList[1].toFloat();
        }
        center.x = data[0];
        center.y = data[1];
        center.z = data[2];
		center.w = 0.f;
        radius = data[3];
        height = data[4];
        read->readLine();
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
            QString tmp = read->readLine();
            QStringList tmpList = tmp.split("=");
            data[i] = tmpList[1].toFloat();
        }
        center.x = data[0];
        center.y = data[1];
        center.z = data[2];
		center.w = 0.f;
        radius = data[3];
        read->readLine();
    }
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {
		cl_float4 length = center - campos;
        float tca = clDot(length, raydir);
		if(tca < 0) return false;
        float dsquare = clDot(length, length) - tca * tca;
		if(dsquare > radius*radius) return false;
		else return true;
	}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {
		cl_float4 length = center - campos;
        float tca = clDot(length, raydir);
        float dsquare = clDot(length, length) - tca * tca;
		float thc = sqrt(radius*radius - dsquare);
        float t = tca - thc;
		cl_float4 intersectPoint = campos + raydir*t;
		return intersectPoint;
	}
private:
    float radius;
};

class Cube : public IShape
{
public:
    Cube(QTextStream* read) {
        float data[6];
        for(int i=0;i<6;i++){
            QString tmp = read->readLine();
            QStringList tmpList = tmp.split("=");
            data[i] = tmpList[1].toFloat();
        }
        center.x = data[0];
        center.y = data[1];
        center.z = data[2];
		center.w = 0.f;
        width = data[3];
        height = data[4];
        depth = data[5];
        read->readLine();
    }
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {
		cl_float4 minpoint, maxpoint;
		minpoint.x = center.x - width/2.f;
		minpoint.y = center.y - height/2.f;
		minpoint.z = center.z - depth/2.f;
		minpoint.w = 0.f;
		
		maxpoint.x = center.x + width/2.f;
		maxpoint.y = center.y + height/2.f;
		maxpoint.z = center.z + depth/2.f;
		maxpoint.w = 0.f;
		
		cl_float4 invdir;
		invdir.x = 1.0f / raydir.x;
		invdir.y = 1.0f / raydir.y;
		invdir.z = 1.0f / raydir.z;
		invdir.w = 0.f;
		
		float t1 = (minpoint.x - campos.x) * invdir.x;
		float t2 = (maxpoint.x - campos.x) * invdir.x;
		float t3 = (minpoint.y - campos.y) * invdir.y;
		float t4 = (maxpoint.y - campos.y) * invdir.y;
		float t5 = (minpoint.z - campos.z) * invdir.z;
		float t6 = (maxpoint.z - campos.z) * invdir.z;
		
        const float tmax = MIN2(MIN2(MAX2(t1, t2), MAX2(t3, t4)), MAX2(t5, t6));
		if(tmax < 0) return false;
		
		
        const float tmin = MAX2(MAX2(MIN2(t1, t2), MIN2(t3, t4)), MIN2(t5, t6));
		if(tmin > tmax) return false;
		
		else return true;
		
	}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {
        cl_float4 minpoint, maxpoint;
        minpoint.x = center.x - width/2.f;
        minpoint.y = center.y - height/2.f;
        minpoint.z = center.z - depth/2.f;
        minpoint.w = 0.f;

        maxpoint.x = center.x + width/2.f;
        maxpoint.y = center.y + height/2.f;
        maxpoint.z = center.z + depth/2.f;
        maxpoint.w = 0.f;

        cl_float4 invdir;
        invdir.x = 1.0f / raydir.x;
        invdir.y = 1.0f / raydir.y;
        invdir.z = 1.0f / raydir.z;
        invdir.w = 0.f;

        float t1 = (minpoint.x - campos.x) * invdir.x;
        float t2 = (maxpoint.x - campos.x) * invdir.x;
        float t3 = (minpoint.y - campos.y) * invdir.y;
        float t4 = (maxpoint.y - campos.y) * invdir.y;
        float t5 = (minpoint.z - campos.z) * invdir.z;
        float t6 = (maxpoint.z - campos.z) * invdir.z;

        const float t = MAX2(MAX2(MIN2(t1, t2), MIN2(t3, t4)), MIN2(t5, t6));
        cl_float4 intersectPoint = campos + raydir*t;
        return intersectPoint;
    }

//http://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection	
private:
    float width;
    float height;
    float depth;
};

#endif // SHAPES_H

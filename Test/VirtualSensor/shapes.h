#ifndef SHAPES_H
#define SHAPES_H

#include "Share/project_common.h"
#include <vector>

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
    Rect(FILE* fp) {}
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {}
private:
    float width;
    float height;
};

class Cylinder : public IShape
{
public:
    Cylinder(FILE* fp) {}
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {}
private:
    float radius;
    float height;
};

class Circle : public IShape
{
public:
    Circle(FILE* fp) {}
    virtual bool DoesRayIntersect(const cl_float4& campos, const cl_float4& raydir) {}
    virtual cl_float4 IntersectingPoint(const cl_float4& campos, const cl_float4& raydir) {}
private:
    float radius;
};

#endif // SHAPES_H

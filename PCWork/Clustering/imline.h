#ifndef IMLINE_H
#define IMLINE_H

#include <vector>
#include "Share/project_common.h"

struct ImLine
{
    cl_int2 endPixels[2];
    float a, b, c;  // ax+by=c

    inline bool IsXAxisMajor() const
    { return (fabsf(a) < fabsf(b)); }

    inline int GetY(const int X) const
    { return (int)((-a*X+c)/b); }

    inline int GetX(const int Y) const
    { return (int)((-b*Y+c)/a); }

    inline void OrthogonalTo(const ImLine& srcLine, const cl_int2 pixel)
    {
        this->a = srcLine.b;
        this->b = -srcLine.a;
        this->c = this->a*pixel.x + this->b*pixel.y;
    }
    inline bool IsAboveLine(const cl_int2& pixel) const
    {
        return (a*pixel.x + b*pixel.y > c);
    }
};

typedef std::vector<ImLine> vecLines;

#endif // IMLINE_H

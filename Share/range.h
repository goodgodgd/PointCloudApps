#ifndef IMRECT_H
#define IMRECT_H

#include "Share/project_common.h"

template<typename T>
struct Range2D
{
    T xl, xh, yl, yh;

    Range2D() {}
    Range2D(T xl_, T xh_, T yl_, T yh_)
        : xl(xl_), xh(xh_), yl(yl_), yh(yh_)
    {}

    Range2D& operator=(const Range2D& srcrect)
    {
        this->xl = srcrect.xl;
        this->xh = srcrect.xh;
        this->yl = srcrect.yl;
        this->yh = srcrect.yh;
        return (*this);
    }

    void ExpandRange(const cl_int2& pixel)
    {
        this->xl = smin(this->xl, pixel.x);
        this->xh = smax(this->xh, pixel.x);
        this->yl = smin(this->yl, pixel.y);
        this->yh = smax(this->yh, pixel.y);
    }

    void ExpandRange(const Range2D<T>& srcRect)
    {
        this->xl = smin(this->xl, srcRect.xl);
        this->xh = smax(this->xh, srcRect.xh);
        this->yl = smin(this->yl, srcRect.yl);
        this->yh = smax(this->yh, srcRect.yh);
    }
    void ReduceRange(const Range2D<T>& srcRect)
    {
        if(srcRect.xh == this->xh && srcRect.xl == this->xl)
        {
            if(this->yh == srcRect.yh)   this->yh = srcRect.yl;
            if(this->yl == srcRect.yl)   this->yl ==srcRect.yh;
        }
        if(srcRect.yh == this->yh && srcRect.yl == this->yl)
        {
            if(this->xh == srcRect.xh)   this->xh = srcRect.xl;
            if(this->xl == srcRect.xl)   this->xl ==srcRect.xh;
        }
}
};

typedef Range2D<int>    ImRect;


template<typename T>
struct Range3D
{
    T xl, xh, yl, yh, zl, zh;

    Range3D() {}
    Range3D(T xl_, T xh_, T yl_, T yh_, T zl_, T zh_)
        : xl(xl_), xh(xh_), yl(yl_), yh(yh_), zl(zl_), zh(zh_)
    {}

    Range3D& operator=(const Range3D& srcrect)
    {
        this->xl = srcrect.xl;
        this->xh = srcrect.xh;
        this->yl = srcrect.yl;
        this->yh = srcrect.yh;
        this->zl = srcrect.zl;
        this->zh = srcrect.zh;
        return (*this);
    }
};


#include <QDebug>
inline QDebug operator <<(QDebug debug, const ImRect &r)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "rect(" << r.xl << ", " << r.xh << ", " << r.yl << ", " << r.yh << ')';
    return debug.space();
}

#endif // IMRECT_H

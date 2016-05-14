#ifndef IMRECT_H
#define IMRECT_H

#include "Share/project_common.h"

template<typename T>
struct Range
{
    T low, high;

    Range() {}
    Range(T low_, T high_)
        : low(low_), high(high_) {}
    Range(T value)
        : low(value), high(value) {}

    Range& operator=(const Range& srcrange)
    {
        this->low = srcrange.low;
        this->high = srcrange.high;
        return (*this);
    }
    Range& operator=(const float srcrange)
    {
        this->low = srcrange;
        this->high = srcrange;
        return (*this);
    }
};

typedef Range<float>    Rangef;

inline Rangef operator -(const float rhs, const Rangef& lhs)
{
    Rangef dst;
    dst.high = rhs - lhs.low;
    dst.low = rhs - lhs.high;
}


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

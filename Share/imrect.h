#ifndef IMRECT_H
#define IMRECT_H

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
    }
};

typedef Range3D<float>    Range3f;


#include <QDebug>

inline QDebug operator <<(QDebug debug, const ImRect &r)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "rect(" << r.xl << ", " << r.xh << ", " << r.yl << ", " << r.yh << ')';
    return debug.space();
}

#endif // IMRECT_H

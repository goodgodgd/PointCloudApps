#ifndef IMRECT_H
#define IMRECT_H

struct ImRect
{
    int xl, xh, yl, yh;

    ImRect() {}
    ImRect(int xl_, int xh_, int yl_, int yh_)
        : xl(xl_), xh(xh_), yl(yl_), yh(yh_)
    {}
};

#endif // IMRECT_H

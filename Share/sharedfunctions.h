#ifndef SHAREDFUNCTIONS_H
#define SHAREDFUNCTIONS_H

#include <QDebug>
#include "imrect.h"

inline QDebug operator <<(QDebug debug, const ImRect &r)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "rect(" << r.xl << ", " << r.xh << ", " << r.yl << ", " << r.yh << ')';
    return debug.space();
}

#endif // SHAREDFUNCTIONS_H

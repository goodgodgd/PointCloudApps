#ifndef TESTNORMALVALIDITY_H
#define TESTNORMALVALIDITY_H

#include <QDebug>
#include <QtGlobal>
#include "Share/project_common.h"
#include "ClUtils/cloperators.h"

namespace Test
{
inline void testNormalValidity(cl_float4* normalCloud)
{
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            Q_ASSERT_X(!clIsNan(normalCloud[IMGIDX(y,x)]), "normal test", "normal is nan");
            float length = clSqLength(normalCloud[IMGIDX(y,x)]);
            Q_ASSERT_X(fabsf(length)<0.0001f || fabsf(length-1.f)<0.0001f, "normal test", "normal is nan");
        }
    }
}

}

#endif // TESTNORMALVALIDITY_H

#ifndef SHAREDENUMS_H
#define SHAREDENUMS_H

namespace ViewOpt
{
    enum Enum
    {
        ViewNone = 0,
        Color = (1<<0),
        Descriptor = (1<<1),
        Segment = (1<<2),
        Object = (1<<3),
        Normal = (1<<4),
        CURVATURE = (1<<5),
        FPFH = (1<<6),
        SHOT = (1<<7)
    };
}

namespace NullID
{
    enum Enum
    {
        NoneNull = 0,
        DescriptorNull,
        NormalNull,
        PointNull
    };
}

#endif // SHAREDENUMS_H

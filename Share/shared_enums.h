#ifndef SHAREDENUMS_H
#define SHAREDENUMS_H

namespace ViewOpt
{
    enum Enum
    {
        ViewNone = 0,
        Color = 2,
        Descriptor = 4,
        Segment = 8,
        Object = 16,
        Normal = 32
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

#ifndef SHAREDENUMS_H
#define SHAREDENUMS_H

namespace ViewOpt
{
    enum eViewOpt
    {
        ViewNone = 0,
        WholeCloud = 1,
        WCColor = 2,
        WCDescriptor = 4,
        WCSegment = 8,
        WCObject = 16,
        Normal = 32
    };
}
#endif // SHAREDENUMS_H

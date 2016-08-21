#ifndef EXPM_STRUCTS_H
#define EXPM_STRUCTS_H

#include "Share/project_common.h"

struct TrackPoint
{
    // sampling
    cl_uint ID;
    cl_uint beginIndex;
    cl_float4 gpoint;
    cl_float4 gnormal;
    // tracking
    cl_uint frameIndex;
    cl_uint2 pixel;
    cl_float4 lpoint;
    cl_uint tcount;
};

struct TrackException
{
    TrackException(QString m) : msg(m) {}
    QString msg;
};

struct EmtpyException
{
    EmtpyException(int code_) : code(code_) {}
    int code;
};


struct RecordException
{
    RecordException(QString m) : msg(m) {}
    QString msg;
};

#endif // EXPM_STRUCTS_H

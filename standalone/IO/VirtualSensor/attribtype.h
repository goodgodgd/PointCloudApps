#ifndef ATTRIBTYPE_H
#define ATTRIBTYPE_H

#include <map>
#include <QString>
#include "Share/project_common.h"

struct AttribData
{
    cl_float4 data;
    AttribData() {}
    AttribData(const cl_float4& datain) : data(datain) {}
    AttribData(const cl_float& datain) { data.s[0] = datain; }
    cl_float4 GetVector() { return data; }
    cl_float GetValue() { return data.s[0]; }
};

typedef std::map<QString, AttribData> MapNameData;
typedef std::pair<QString, AttribData> PairNameData;

#endif // ATTRIBTYPE_H

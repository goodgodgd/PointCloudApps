#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include <vector>
#include <map>
#include "Share/project_common.h"
#include "Share/imrect.h"

typedef std::pair<int, int>             PairOfInts;
typedef std::vector<PairOfInts>         vecPairOfInts;
typedef std::map<int,int>               mapPairOfInts;
typedef std::pair<cl_int2, cl_int2>     PairOfPixels;
typedef std::vector<PairOfPixels>       vecPairOfPixels;
typedef std::pair<cl_float4, cl_float4> PairPointNormal;
typedef std::pair<cl_float4, cl_float4> PairOfPoints;
typedef std::vector<cl_int2>            vecPixels;

#endif // SHARED_TYPES_H

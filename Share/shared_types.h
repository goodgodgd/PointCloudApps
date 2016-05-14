#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include <vector>
#include <map>
#include "Share/project_common.h"
#include "Share/range.h"

typedef std::vector<int>                vecInts;
typedef std::pair<int, int>             PairOfInts;
typedef std::map<int,int>               mapPairOfInts;
typedef std::vector<PairOfInts>         vecPairOfInts;
typedef std::vector<vecInts>            vecOfVecInts;

typedef std::vector<float>              vecFloats;
typedef std::pair<float, float>         PairOfFloats;
typedef std::vector<PairOfFloats>       vecPairOfFloats;

typedef std::vector<cl_int2>            vecPixels;
typedef std::pair<cl_int2, cl_int2>     PairOfPixels;
typedef std::vector<PairOfPixels>       vecPairOfPixels;

typedef std::pair<cl_float4, cl_float4> PairPointNormal;
typedef std::pair<cl_float4, cl_float4> PairOfPoints;
typedef std::vector<PairOfPoints>       vecPairOfPoints;

#endif // SHARED_TYPES_H

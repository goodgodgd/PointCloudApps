#ifndef POINTNORMALSMOOTHER_H
#define POINTNORMALSMOOTHER_H

#include "Share/project_common.h"
#include "ClUtils/cloperators.h"
#include "indexsort.h"

#define TESTNORMALSMOOTHER
#include "Test/testnormalsmoother.h"

class NormalSmoother
{
#define ADJ_SIZE    7
#define MG          2

public:
    NormalSmoother();
    static void SmootheNormalCloud(cl_float4* pointCloud, cl_float4* normalCloud);
    static void SetTester(TestNormalSmoother* tester_in);
private:
    static int GetAdjacentNormals(cl_float4* pointCloud, cl_float4* normalCloud, cl_int2 pixel, cl_float4* adjacentNormals);
    static cl_float4 AverageInlierNormals(cl_float4* adjacentNormals, const int numNormals);
    inline static cl_float4 AverageNormals(cl_float4* normals, const int num);
    inline static bool AngleBetweenVectorsLargerThan(const cl_float4& v1, const cl_float4& v2, const float degree, bool b_normalized);
    static TestNormalSmoother* tester;
};

#endif // POINTNORMALSMOOTHER_H

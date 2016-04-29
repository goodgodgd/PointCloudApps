#ifndef NORMALMAKER_H
#define NORMALMAKER_H

#include <QElapsedTimer>
#include "Share/project_common.h"
#include "ClUtils/cl_macros.h"
#include "ClUtils/clsetup.h"
#include "ClUtils/cl_utils.h"
#include "ClUtils/clbase.h"

class NormalMaker : public ClBase
{
public:
    NormalMaker();
    ~NormalMaker();
    void ComputeNormal(cl_mem memPoints, cl_mem memNeighborIndices, cl_mem memNumNeighbors, cl_int maxNeighbors
                       , cl_float4* normalCloud_out);
    cl_mem memNormals;

private:
    void Setup();
};

#endif // NORMALMAKER_H

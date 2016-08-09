#ifndef DESCGRADIENTMAKER_H
#define DESCGRADIENTMAKER_H

#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "Share/arraydata.h"
#include "ClUtils/cl_macros.h"
#include "ClUtils/clsetup.h"
#include "ClUtils/cl_utils.h"
#include "ClUtils/clbase.h"


class DescGradientMaker : public ClBase
{
public:
    DescGradientMaker();
    ~DescGradientMaker();
    void ComputeGradient(cl_mem memPoints, cl_mem memNormals, cl_mem memNeighborIndices, cl_mem memNumNeighbors
                           , cl_int maxNeighbors, cl_mem memDescriptors);
    DescType* GetDescriptor();

private:
    void Setup();
    cl_int szDescriptors;
    ArrayData<DescType> descriptorData;
};

#endif // DESCGRADIENTMAKER_H

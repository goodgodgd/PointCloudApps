#ifndef RADIUSSEARCH_H
#define RADIUSSEARCH_H

#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/forsearchneigbhor.h"
#include "Share/arraydata.h"
#include "ClUtils/cl_macros.h"
#include "ClUtils/clsetup.h"
#include "ClUtils/cl_utils.h"
#include "ClUtils/clbase.h"

class RadiusSearch : public ClBase
{
public:
    RadiusSearch();
    ~RadiusSearch();
    void SearchNeighborIndices(cl_float4* srcPointCloud, cl_float radiusMeter, cl_float focalLength, cl_int maxNeighbors);
    cl_int* GetNeighborIndices();
    cl_int* GetNumNeighbors();

    cl_mem memPoints;
    cl_mem memNeighborIndices;
    cl_mem memNumNeighbors;
    cl_int szNeighborIdcs;
    cl_int szNumNeighbors;
    ArrayData<cl_int> neibIndicesData;
    ArrayData<cl_int> numNeibsData;

private:
    void Setup();
};

#endif // RADIUSSEARCH_H

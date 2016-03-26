#ifndef RADIUSSEARCH_H
#define RADIUSSEARCH_H

#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/forsearchneigbhor.h"
#include "ClUtils/cl_macros.h"
#include "ClUtils/clsetup.h"
#include "ClUtils/cl_utils.h"

class RadiusSearch
{
public:
    RadiusSearch();
    ~RadiusSearch();
    void SearchNeighborIndices(cl_float4* srcPointCloud, cl_float radiusMeter, cl_float focalLength, cl_int maxNeighbors
                              , cl_int* neighborIndices_out, cl_int* numNeighbors_out);

    cl_mem memPoints;
    cl_mem memNeighborIndices;
    cl_mem memNumNeighbors;
    cl_float debugBuffer[DEBUG_FL_SIZE];

private:
    void Setup();

    bool b_init;
    cl_device_id device;
    cl_context context;
    cl_command_queue queue;
    cl_program program;
    cl_kernel kernel;
    size_t gwsize[2];
    size_t lwsize[2];
    size_t imgOrigin[3];
    size_t imgRegion[3];
    cl_int szNeighborIdcs;
    cl_int szNumNeighbors;
    cl_int szDebug;
    cl_mem memDebug;
};

#endif // RADIUSSEARCH_H

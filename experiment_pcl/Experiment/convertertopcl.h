#ifndef CONVERTERTOPCL_H
#define CONVERTERTOPCL_H

#include <QDebug>
#include "Share/project_common.h"
#include "Share/shared_data.h"
#include "Share/exceptions.h"
#include "Share/shared_enums.h"
#include "ShareExpm/expm_common.h"
#include "ShareExpm/expm_structs.h"

class ConverterToPcl
{
public:
    ConverterToPcl();

    void ConvertToPCLPointCloud(SharedData* shdDat, const bool bFilter=true);
    void ConvertToPCLPointVector(SharedData* shdDat);

    VoxelCloud::Ptr pclPointCloud;
    NormalCloud::Ptr pclNormalCloud;
    VectorVoxel pclPoints;
    VectorNormal pclNormals;
};

#endif // CONVERTERTOPCL_H

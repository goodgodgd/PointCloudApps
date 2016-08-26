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

    VoxelCloud::Ptr GetPointCloud() { return pclPointCloud; }
    NormalCloud::Ptr GetNormalCloud() { return pclNormalCloud; }
    std::vector<NormalCloud::Ptr> GetThreeAxesCloud()
    {
        std::vector<NormalCloud::Ptr> threeAxes;
        threeAxes.push_back(pclNormalCloud);
        threeAxes.push_back(pclMaxPrincAxes);
        threeAxes.push_back(pclMinPrincAxes);

        return threeAxes;
    }

    VoxelCloud::Ptr pclPointCloud;
    NormalCloud::Ptr pclNormalCloud;
    NormalCloud::Ptr pclMaxPrincAxes;
    NormalCloud::Ptr pclMinPrincAxes;
    VectorVoxel pclPoints;
    VectorNormal pclNormals;
};

#endif // CONVERTERTOPCL_H

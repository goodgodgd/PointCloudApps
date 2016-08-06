#ifndef EXPM_COMMON_H
#define EXPM_COMMON_H

#include <vector>
#include <pcl/common/common_headers.h>
#include "Share/forsearchneigbhor.h"

typedef pcl::PointXYZ               VoxelType;
typedef pcl::PointCloud<VoxelType>  VoxelCloud;
typedef std::vector<VoxelType>      VectorVoxel;

typedef pcl::Normal                 NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef std::vector<VoxelType>      VectorNormal;   // gpu modules use PointXYZ for normal vector

#define SPIN_SIZE                   153
typedef pcl::Histogram<SPIN_SIZE>   SpinImageType;
typedef pcl::FPFHSignature33        FPFHType;
typedef pcl::SHOT352                SHOTType;
typedef pcl::Narf36                 NarfType;

namespace GpuSel
{
enum Enum
{
    CWG=1,
    SPIN=2,
    FPFH=4,
};
}

#define OBJ_PIXEL_ITV               2

extern float g_descriptorRadius;


#endif // EXPM_COMMON_H

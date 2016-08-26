#ifndef EXPM_COMMON_H
#define EXPM_COMMON_H

#include <vector>
#include <pcl/common/common_headers.h>

typedef pcl::PointXYZ               VoxelType;
typedef pcl::PointCloud<VoxelType>  VoxelCloud;
typedef std::vector<VoxelType>      VectorVoxel;

typedef pcl::Normal                 NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef std::vector<VoxelType>      VectorNormal;   // gpu modules use PointXYZ for normal vector

#define SPIN_SIZE                   153
typedef pcl::FPFHSignature33        FPFHType;
typedef pcl::SHOT352                SHOTType;
typedef pcl::Narf36                 NarfType;
typedef pcl::Histogram<SPIN_SIZE>   SpinImageType;
typedef pcl::Histogram<SPIN_SIZE*3> TrisiType;

namespace GpuSel
{
enum Enum
{
    CWG=1,
    SPIN=2,
    FPFH=4,
};
}

#endif // EXPM_COMMON_H

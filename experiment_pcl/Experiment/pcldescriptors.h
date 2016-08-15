#ifndef PCLDESCRIPTORS_H
#define PCLDESCRIPTORS_H

#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "ClUtils/cloperators.h"
#include "ShareExpm/expm_common.h"
#include "ShareExpm/expm_structs.h"
#include "Features/pclcpufeatures.h"
#include "Features/pclgpufeatures.h"
#include "convertertopcl.h"

class PclDescriptors
{
public:
    PclDescriptors();
    void ComputeWholeDescriptors(SharedData* shdDat, const int gpuSelect, const float descriptorRadius, const int maxNeighbors);
    void ComputeObjectDescriptors(SharedData* shdDat, const float descriptorRadius);
    void ComputeTrackingDescriptors(SharedData* shdDat, const std::vector<TrackPoint>* trackPoints, const float descriptorRadius);

    pcl::PointCloud<SpinImageType>::Ptr GetSpinImage()
    {
        if(gpuSelect & GpuSel::SPIN)
            return spin_gpu.descriptors;
        else
            return spin_cpu.descriptors;
    }
    pcl::PointCloud<FPFHType>::Ptr GetFpfh()
    {
        if(gpuSelect & GpuSel::FPFH)
            return fpfh_gpu.descriptors;
        else
            return fpfh_cpu.descriptors;
    }
    pcl::PointCloud<SHOTType>::Ptr GetShot()
    {
        return shot_cpu.descriptors;
    }

    boost::shared_ptr<std::vector<int>> indicesptr;

private:
    void ComputeIndexedDescriptors(SharedData* shdDat, const int gpuSel, const float descriptorRadius
                                   , boost::shared_ptr<std::vector<int>> indicesptr);
    ConverterToPcl pclConverter;

    CpuFeature::FpfhEstimator fpfh_cpu;
    CpuFeature::SpinImageEstimator spin_cpu;
    CpuFeature::ShotEstimator shot_cpu;
    CpuFeature::NarfEstimator narf_cpu;

    GpuFeature::FpfhEstimator fpfh_gpu;
    GpuFeature::SpinImageEstimator spin_gpu;

    QElapsedTimer eltimer;
    int gpuSelect;
};

#endif // PCLDESCRIPTORS_H

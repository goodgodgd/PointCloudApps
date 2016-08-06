#ifndef PCLGPUFEATURES_H
#define PCLGPUFEATURES_H

#include <QDebug>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/gpu/features/features.hpp>
#include "Share/project_common.h"
#include "Share/exceptions.h"
#include "Share/shared_enums.h"
#include "Share/forsearchneigbhor.h"
#include "ShareExpm/expm_common.h"

namespace GpuFeature
{

class FpfhEstimator
{
public:
    FpfhEstimator() {}
    pcl::PointCloud<FPFHType>::Ptr descriptors;

    void EstimateFpfh(VectorVoxel& points, VectorNormal& normals, const uchar* nullityMap)
    {
        std::vector<FPFHType> descrs;
        descrs.clear();
        pcl::gpu::DeviceArray<VoxelType> gpupoints;
        pcl::gpu::DeviceArray<VoxelType> gpunormals;
        gpupoints.upload(points);
        gpunormals.upload(normals);

        // FPFH estimation object.
        pcl::gpu::FPFHEstimation fpfh;
        pcl::gpu::DeviceArray2D<FPFHType> gpuDescriptors;
        fpfh.setInputCloud(gpupoints);
        fpfh.setInputNormals(gpunormals);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        fpfh.setRadiusSearch(g_descriptorRadius, NEIGHBORS_PER_POINT);

        fpfh.compute(gpuDescriptors);

        int cols=0;
        gpuDescriptors.download(descrs, cols);

    //    {
    //        QDebug dbg = qDebug();
    //        dbg << "descriptorsByCpu GPU" << descrs.size();
    //        for(int i=0; i<FPFHType::descriptorSize(); i++)
    //            dbg << descrs[100].histogram[i];
    //    }

        FPFHType zeroPad;
        for(int i=0; i<FPFHType::descriptorSize(); i++)
            zeroPad.histogram[i] = 0.f;
        for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
        {
            if(nullityMap[i]!=NullID::NoneNull)
            {
                if(i<descrs.size())
                    descrs.insert(descrs.begin()+i, zeroPad);
                else
                    descrs.push_back(zeroPad);
            }
        }


        if(descrs.size() != IMAGE_WIDTH*IMAGE_HEIGHT)
            throw TryFrameException(QString("fpfh gpu output is not valid %1").arg(descrs.size()));

        descriptors->points.clear();
        descriptors->points.insert(descriptors->points.begin(), descrs.begin(), descrs.end());
    }
};


class SpinImageEstimator
{
public:
    SpinImageEstimator() {}
    pcl::PointCloud<SpinImageType>::Ptr descriptors;

    void EstimateSpinImage(VectorVoxel& points, VectorNormal& normals, const uchar* nullityMap)
    {
        std::vector<SpinImageType> descrs;

        pcl::gpu::DeviceArray<VoxelType> gpupoints;
        pcl::gpu::DeviceArray<VoxelType> gpunormals;
        gpupoints.upload(points);
        gpunormals.upload(normals);

        // FPFH estimation object.
        pcl::gpu::SpinImageEstimation spinImage;
        pcl::gpu::DeviceArray2D<SpinImageType> gpuDescriptors;
        pcl::gpu::DeviceArray<uchar> gpuMask;
        spinImage.setInputWithNormals(gpupoints, gpunormals);
    //    spinImage.setInputCloud(gpupoints);
    //    spinImage.setInputNormals(gpunormals);
        spinImage.useNormalsAsRotationAxis();
    //    spinImage.setMinPointCountInNeighbourhood(20);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        spinImage.setRadiusSearch(g_descriptorRadius, NEIGHBORS_PER_POINT);
        qDebug() << "ComputeSpinImageByGPU3";
        spinImage.compute(gpuDescriptors, gpuMask);
        qDebug() << "ComputeSpinImageByGPU4";

        int cols=0;
        gpuDescriptors.download(descrs, cols);

//        {
//            QDebug dbg = qDebug();
//            dbg << "SpinImage by GPU" << descrs.size();
//            for(int i=0; i<SpinImageType::descriptorSize(); i++)
//                dbg << descrs[10000].histogram[i];
//        }

        SpinImageType zeroPad;
        for(int i=0; i<SPIN_SIZE; i++)
            zeroPad.histogram[i] = 0.f;
        for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
        {
            if(nullityMap[i]!=NullID::NoneNull)
            {
                if(i<descrs.size())
                    descrs.insert(descrs.begin()+i, zeroPad);
                else
                    descrs.push_back(zeroPad);
            }
        }

        if(descrs.size() != IMAGE_WIDTH*IMAGE_HEIGHT)
            throw TryFrameException(QString("spin gpu output is not valid %1").arg(descrs.size()));

        descriptors->points.clear();
        descriptors->points.insert(descriptors->points.begin(), descrs.begin(), descrs.end());
    }
};


class PrincipleCurvatureEstimator
{
public:
    PrincipleCurvatureEstimator() {}
    std::vector<pcl::PrincipalCurvatures> descrs;

    void EstimatePrincipalCurvature(VectorVoxel& points, VectorNormal& normals)
    {

        pcl::gpu::DeviceArray<VoxelType> gpupoints;
        pcl::gpu::DeviceArray<VoxelType> gpunormals;
        gpupoints.upload(points);
        gpunormals.upload(normals);

        // FPFH estimation object.
        pcl::gpu::PrincipalCurvaturesEstimation prinCurv;
        pcl::gpu::DeviceArray<pcl::PrincipalCurvatures> gpuDescriptors;
        prinCurv.setInputCloud(gpupoints);
        prinCurv.setInputNormals(gpunormals);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        prinCurv.setRadiusSearch(g_descriptorRadius, NEIGHBORS_PER_POINT);
        prinCurv.compute(gpuDescriptors);

        gpuDescriptors.download(descrs);
    }
};

}


#endif // PCLGPUFEATURES_H

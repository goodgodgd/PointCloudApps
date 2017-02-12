#ifndef PCLCPUFEATURES_H
#define PCLCPUFEATURES_H

#include <QDebug>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/shot.h>
#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/range_image/range_image_planar.h>
#include <Eigen/Eigen>
#include "Share/project_common.h"
#include "Share/camera_param.h"
#include "Share/exceptions.h"
#include "Share/shared_enums.h"
#include "Share/fordescriptor.h"
#include "ShareExpm/expm_common.h"

namespace CpuFeature
{

template <typename CloudType, typename PointType>
void ZeroPadding(CloudType descriptors, const uchar* nullityMap, PointType zeroPad)
{
    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
    {
        if(nullityMap[i]!=NullID::NoneNull)
        {
            if(i<descriptors->points.size())
                descriptors->points.insert(descriptors->points.begin()+i, zeroPad);
            else
                descriptors->points.push_back(zeroPad);
        }
    }
    qDebug() << "zero padded";
}

class FpfhEstimator
{
public:
    FpfhEstimator()
        : descriptors(new pcl::PointCloud<FPFHType>())
    {}
    pcl::PointCloud<FPFHType>::Ptr descriptors;

    void EstimateFpfh(VoxelCloud::Ptr points, NormalCloud::Ptr normals, const uchar* nullityMap, const float descriptorRadius
                      , boost::shared_ptr<std::vector<int>> indicesptr=nullptr)
    {
        descriptors->clear();
        pcl::search::KdTree<VoxelType>::Ptr tree(new pcl::search::KdTree<VoxelType>);
        // FPFH estimation object.
        pcl::FPFHEstimation<VoxelType, NormalType, FPFHType> fpfh;
        fpfh.setInputCloud(points);
        fpfh.setInputNormals(normals);
        fpfh.setSearchMethod(tree);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        fpfh.setRadiusSearch(descriptorRadius);
        if(indicesptr!=nullptr)
            fpfh.setIndices(indicesptr);

        qDebug() << "fpfh input" << points->size() << normals->size() << indicesptr->size() << descriptors->size()
                 << points->at(IMGIDX(100,100)).x << normals->at(IMGIDX(100,100)).normal_x;
        fpfh.compute(*descriptors);
        qDebug() << "fpfh computed";

    //    {
    //        QDebug dbg = qDebug();
    //        dbg << "descriptors CPU" << descriptors->points.size();
    //        for(int i=0; i<FPFHType::descriptorSize(); i++)
    //            dbg << descriptors->points[100].histogram[i];
    //    }

        if(indicesptr==nullptr)
        {
            FPFHType zeroPad;
            for(int i=0; i<FPFHType::descriptorSize(); i++)
                zeroPad.histogram[i] = 0.f;
            ZeroPadding<pcl::PointCloud<FPFHType>::Ptr, FPFHType>(descriptors, nullityMap, zeroPad);
            if(descriptors->points.size() != IMAGE_WIDTH*IMAGE_HEIGHT)
                throw TryFrameException(QString("fpfh cpu output is not valid %1").arg(descriptors->points.size()));
        }
        else
        {
            if(descriptors->points.size() != indicesptr->size())
                throw TryFrameException(QString("fpfh cpu selected is not valid %1 %2").arg(descriptors->points.size()).arg(indicesptr->size()));
        }
    }
};


class SpinImageEstimator
{
public:
    SpinImageEstimator()
        : descriptors(new pcl::PointCloud<SpinImageType>())
    {}
    pcl::PointCloud<SpinImageType>::Ptr descriptors;

    void EstimateSpinImage(VoxelCloud::Ptr points, NormalCloud::Ptr normals, const uchar* nullityMap, const float descriptorRadius
                           , boost::shared_ptr<std::vector<int>> indicesptr=nullptr)
    {
        descriptors->clear();
        pcl::search::KdTree<VoxelType>::Ptr tree(new pcl::search::KdTree<VoxelType>);
        // FPFH estimation object.
        pcl::SpinImageEstimation<VoxelType, NormalType, SpinImageType> spinImage;
        spinImage.setInputCloud(points);
        spinImage.setInputNormals(normals);
        spinImage.setSearchMethod(tree);
        spinImage.useNormalsAsRotationAxis();
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        spinImage.setRadiusSearch(descriptorRadius);
        if(indicesptr!=nullptr)
            spinImage.setIndices(indicesptr);

        spinImage.compute(*descriptors);


        if(indicesptr==nullptr)
        {
            SpinImageType zeroPad;
            for(int i=0; i<SpinImageType::descriptorSize(); i++)
                zeroPad.histogram[i] = 0.f;
            ZeroPadding<pcl::PointCloud<SpinImageType>::Ptr, SpinImageType>(descriptors, nullityMap, zeroPad);

            if(descriptors->points.size() != IMAGE_WIDTH*IMAGE_HEIGHT)
                throw TryFrameException(QString("spinImage cpu output is not valid %1").arg(descriptors->points.size()));
        }
        else
        {
            if(descriptors->points.size() != indicesptr->size())
                throw TryFrameException(QString("spinImage cpu output is not valid %1").arg(descriptors->points.size()));
        }
    }
};

class ShotEstimator
{
public:
    ShotEstimator()
        : descriptors(new pcl::PointCloud<SHOTType>())
    {}
    pcl::PointCloud<SHOTType>::Ptr descriptors;

    void EstimateShot(VoxelCloud::Ptr points, NormalCloud::Ptr normals, const uchar* nullityMap, const float descriptorRadius
                      , boost::shared_ptr<std::vector<int>> indicesptr=nullptr)
    {
        descriptors->clear();
        pcl::search::KdTree<VoxelType>::Ptr tree(new pcl::search::KdTree<VoxelType>);
        // FPFH estimation object.
        pcl::SHOTEstimation<VoxelType, NormalType, SHOTType> shot;
        shot.setInputCloud(points);
        shot.setInputNormals(normals);
        shot.setSearchMethod(tree);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        shot.setRadiusSearch(descriptorRadius);
        if(indicesptr!=nullptr)
            shot.setIndices(indicesptr);

        shot.compute(*descriptors);

        if(indicesptr==nullptr)
        {
            SHOTType zeroPad;
            for(int i=0; i<SHOTType::descriptorSize(); i++)
                zeroPad.descriptor[i] = 0.f;
            ZeroPadding<pcl::PointCloud<SHOTType>::Ptr, SHOTType>(descriptors, nullityMap, zeroPad);

            if(descriptors->points.size() != IMAGE_WIDTH*IMAGE_HEIGHT)
                throw TryFrameException(QString("shot cpu output is not valid %1").arg(descriptors->points.size()));
        }
        else
        {
            if(descriptors->points.size() != indicesptr->size())
                throw TryFrameException(QString("shot cpu output is not valid %1").arg(descriptors->points.size()));
        }
    }
};


class TrisiEstimator
{
public:
    TrisiEstimator()
        : descriptors(new pcl::PointCloud<TrisiType>())
    {}
    pcl::PointCloud<TrisiType>::Ptr descriptors;
    SpinImageEstimator spinEstimator;

    void EstimateTrisi(VoxelCloud::Ptr points, std::vector<NormalCloud::Ptr> threeAxes, const uchar* nullityMap, const float descriptorRadius
                       , boost::shared_ptr<std::vector<int>> indicesptr=nullptr)
    {
        descriptors->clear();
        if(indicesptr==nullptr)
            descriptors->resize(IMAGE_WIDTH*IMAGE_HEIGHT);
        else
            descriptors->resize(indicesptr->size());
        const int spinSize = SpinImageType::descriptorSize();

        for(size_t ai=0; ai<threeAxes.size(); ++ai)
        {
            spinEstimator.EstimateSpinImage(points, threeAxes[ai], nullityMap, descriptorRadius, indicesptr);
            if(descriptors->size() != spinEstimator.descriptors->size())
                throw TryFrameException("sizes of trisi and spin image disagree");
            // set Spin Image into TriSI
            for(int pi=0; pi<(int)descriptors->size(); ++pi)
            {
                for(int di=0; di<spinSize; ++di)
                    descriptors->at(pi).histogram[spinSize*ai + di] = spinEstimator.descriptors->at(pi).histogram[di];
            }
        }
    }
};

class PrincipleCurvatureEstimator
{
public:
    PrincipleCurvatureEstimator()
        : descriptors(new pcl::PointCloud<pcl::PrincipalCurvatures>())
    {}
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr descriptors;

    void EstimatePrincipalCurvature(VoxelCloud::Ptr points, NormalCloud::Ptr normals)
    {
        pcl::PrincipalCurvaturesEstimation<VoxelType, pcl::Normal, pcl::PrincipalCurvatures> prinCurv;
        pcl::search::KdTree<VoxelType>::Ptr tree (new pcl::search::KdTree<VoxelType>);

        prinCurv.setInputCloud(points);
        prinCurv.setInputNormals(normals);
        prinCurv.setSearchMethod(tree);
        prinCurv.setRadiusSearch(1.0);
        prinCurv.compute(*descriptors);
    }
};

class NarfEstimator
{
public:
    NarfEstimator()
        : descriptors(new pcl::PointCloud<NarfType>())
    {}
    pcl::PointCloud<NarfType>::Ptr descriptors;

    void EstimateNarf(VoxelCloud::Ptr points, NormalCloud::Ptr normals, const uchar* nullityMap, boost::shared_ptr<std::vector<int>> indicesptr=nullptr)
    {
        descriptors->clear();
        // Convert the cloud to range image.
        int imageSizeX = IMAGE_WIDTH, imageSizeY = IMAGE_HEIGHT;
        float centerX = (IMAGE_WIDTH / 2.0f), centerY = (IMAGE_HEIGHT / 2.0f);
        float focalLengthX = CameraParam::flh(), focalLengthY = CameraParam::flv();
        float noiseLevel = 0.0f, minimumRange = 0.0f;
        Eigen::Affine3f sensorPose;
        sensorPose.setIdentity();
        pcl::RangeImagePlanar rangeImage;
        rangeImage.createFromPointCloudWithFixedSize(*points, imageSizeX, imageSizeY,
                centerX, centerY, focalLengthX, focalLengthX,
                sensorPose, pcl::RangeImage::CAMERA_FRAME,
                noiseLevel, minimumRange);

        std::vector<int> keypoints;
        if(indicesptr!=nullptr)
        {
            keypoints.resize(indicesptr->size());
            for(int i=0; i<indicesptr->size(); ++i)
                keypoints[i] = indicesptr->at(i);
        }
        else
        {
            keypoints.resize(points->size());
            for(int i=0; i<points->size(); ++i)
                keypoints[i] = i;
        }

        // NARF estimation object.
        pcl::NarfDescriptor narf(&rangeImage, &keypoints);
        narf.getParameters().support_size = 0.04f;
        // If true, the rotation invariant version of NARF will be used. The histogram
        // will be shifted according to the dominant orientation to provide robustness to
        // rotations around the normal.
        narf.getParameters().rotation_invariant = true;
        narf.compute(*descriptors);

        qDebug() << "narf size" << descriptors->points.size();

        if(indicesptr==nullptr)
        {
            NarfType zeroPad;
            for(int i=0; i<NarfType::descriptorSize(); i++)
                zeroPad.descriptor[i] = 0.f;
            for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
            {
                if(nullityMap[i]!=NullID::NoneNull)
                {
                    if(i<descriptors->points.size())
                        descriptors->points.insert(descriptors->points.begin()+i, zeroPad);
                    else
                        descriptors->points.push_back(zeroPad);
                }
            }
        }

        if(indicesptr!=nullptr)
        {
            if(descriptors->points.size() != indicesptr->size())
                throw TryFrameException(QString("narf cpu output is not valid %1").arg(descriptors->points.size()));
        }
        else if(descriptors->points.size() != IMAGE_WIDTH*IMAGE_HEIGHT)
            throw TryFrameException(QString("narf cpu output is not valid %1").arg(descriptors->points.size()));
    }
};

}

#endif // PCLCPUFEATURES_H

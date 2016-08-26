#include "convertertopcl.h"

ConverterToPcl::ConverterToPcl()
    : pclPointCloud(new VoxelCloud)
    , pclNormalCloud(new NormalCloud)
    , pclMaxPrincAxes(new NormalCloud)
    , pclMinPrincAxes(new NormalCloud)
{
}

void ConverterToPcl::ConvertToPCLPointCloud(SharedData* shdDat, const bool bFilter)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const AxesType* prinAxes = shdDat->ConstPrinAxes();
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();

    VoxelType voxel;
    NormalType normal;
    pclPointCloud->clear();
    pclNormalCloud->clear();
    pclMaxPrincAxes->clear();
    pclMinPrincAxes->clear();

    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
    {
        if(bFilter && nullityMap[i]!=NullID::NoneNull)
            continue;
        voxel.getArray4fMap() << pointCloud[i].x, pointCloud[i].y, pointCloud[i].z, 0;
        pclPointCloud->push_back(voxel);

        normal.getNormalVector4fMap() << normalCloud[i].x, normalCloud[i].y, normalCloud[i].z, 0;
        pclNormalCloud->push_back(normal);

        normal.getNormalVector4fMap() << prinAxes[i].s[0], prinAxes[i].s[1], prinAxes[i].s[2], 0;
        pclMaxPrincAxes->push_back(normal);

        normal.getNormalVector4fMap() << prinAxes[i].s[4], prinAxes[i].s[5], prinAxes[i].s[6], 0;
        pclMinPrincAxes->push_back(normal);
    }
    qDebug() << "pcl cloud size" << pclPointCloud->points.size();

//    qDebug() << "count" << count << pclPointCloud->size() << pclNormalCloud->size()
//                << "point" << pclPointCloud->points[100].x << pclPointCloud->points[100].y << pclPointCloud->points[100].z
//                   << "normal" << pclNormalCloud->points[100].normal_x << pclNormalCloud->points[100].normal_y << pclNormalCloud->points[100].normal_z;
}

void ConverterToPcl::ConvertToPCLPointVector(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();

    VoxelType voxel;
    VoxelType normal;
    pclPoints.clear();
    pclNormals.clear();

    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
    {
        if(nullityMap[i]!=NullID::NoneNull)
            continue;
        voxel.getArray4fMap() << pointCloud[i].x, pointCloud[i].y, pointCloud[i].z, 0;
        pclPoints.push_back(voxel);
        normal.getArray4fMap() << normalCloud[i].x, normalCloud[i].y, normalCloud[i].z, 0;
        pclNormals.push_back(normal);
    }    
    qDebug() << "pcl vector size" << pclPoints.size();

//    if(pclPoints.size() != IMAGE_WIDTH*IMAGE_HEIGHT)
//        throw TryFrameException(QString("insufficient point cloud %1").arg(pclPoints.size()));
}

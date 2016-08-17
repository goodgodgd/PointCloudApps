#include "pcldescriptors.h"

PclDescriptors::PclDescriptors()
    : gpuSelect(0)
    , indicesptr (new std::vector<int>)
{
}

void PclDescriptors::ComputeWholeDescriptors(SharedData* shdDat, const int gpuSel, const float descriptorRadius, const int maxNeighbors)
{
    qDebug() << "---------- PCL descriptors ----------";
    pclConverter.ConvertToPCLPointCloud(shdDat);
    gpuSelect = gpuSel;
    if(gpuSelect!=0)
        pclConverter.ConvertToPCLPointVector(shdDat);
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();

    eltimer.start();
    if(gpuSelect & GpuSel::SPIN)
        spin_cpu.EstimateSpinImage(pclConverter.pclPointCloud, pclConverter.pclNormalCloud, nullityMap, descriptorRadius);
    else
        spin_gpu.EstimateSpinImage(pclConverter.pclPoints, pclConverter.pclNormals, nullityMap, descriptorRadius, maxNeighbors);
    qDebug() << "SpinImage took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    if(gpuSelect & GpuSel::FPFH)
        fpfh_cpu.EstimateFpfh(pclConverter.pclPointCloud, pclConverter.pclNormalCloud, nullityMap, descriptorRadius);
    else
        fpfh_gpu.EstimateFpfh(pclConverter.pclPoints, pclConverter.pclNormals, nullityMap, descriptorRadius, maxNeighbors);
    qDebug() << "FPFH took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    shot_cpu.EstimateShot(pclConverter.pclPointCloud, pclConverter.pclNormalCloud, nullityMap, descriptorRadius);
    qDebug() << "SHOT took" << eltimer.nsecsElapsed()/1000 << "us";

//    eltimer.start();
//    narf_cpu.EstimateNarf(pclConverter.pclPointCloud, pclConverter.pclNormalCloud, nullityMap);
//    qDebug() << "NARF took" << eltimer.nsecsElapsed()/1000 << "us";
}

void PclDescriptors::ComputeObjectDescriptors(SharedData* shdDat, const float descriptorRadius)
{
    qDebug() << "---------- PCL Object Descriptors ----------";
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();
    const int indicSizeUpto = 1000;
    int pxitv = 1;

    do {
        pxitv++;
        indicesptr->clear();
        for(int y=0; y<IMAGE_HEIGHT; y+=pxitv)
            for(int x=0; x<IMAGE_WIDTH; x+=pxitv)
                if(nullityMap[IMGIDX(y,x)] == NullID::NoneNull)
                    indicesptr->push_back(IMGIDX(y,x));
        qDebug() << "object indices size" << indicesptr->size() << indicesptr->at(0) << pxitv;
    } while (indicesptr->size() > indicSizeUpto);

    ComputeIndexedDescriptors(shdDat, 0, descriptorRadius, indicesptr);
}

void PclDescriptors::ComputeTrackingDescriptors(SharedData* shdDat, const std::vector<TrackPoint>* trackPoints, const float descriptorRadius)
{
    qDebug() << "---------- PCL Tracking Descriptors ----------";
    indicesptr->clear();
    for(size_t i=0; i<trackPoints->size(); i++)
    {
        if(trackPoints->at(i).frameIndex == g_frameIdx)
        {
            int idx = IMGIDX(trackPoints->at(i).pixel.y, trackPoints->at(i).pixel.x);
            indicesptr->push_back(idx);
        }
    }
    ComputeIndexedDescriptors(shdDat, 0, descriptorRadius, indicesptr);
}

void PclDescriptors::ComputeIndexedDescriptors(SharedData* shdDat, const int gpuSel, const float descriptorRadius
                                               , boost::shared_ptr<std::vector<int>> indicesptr)
{
    const bool bFilter = false;
    pclConverter.ConvertToPCLPointCloud(shdDat, bFilter);
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();
    gpuSelect = 0;

    eltimer.start();
    fpfh_cpu.EstimateFpfh(pclConverter.pclPointCloud, pclConverter.pclNormalCloud, nullityMap, descriptorRadius, indicesptr);
    qDebug() << "FPFH CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    spin_cpu.EstimateSpinImage(pclConverter.pclPointCloud, pclConverter.pclNormalCloud, nullityMap, descriptorRadius, indicesptr);
    qDebug() << "SpinImage CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    shot_cpu.EstimateShot(pclConverter.pclPointCloud, pclConverter.pclNormalCloud, nullityMap, descriptorRadius, indicesptr);
    qDebug() << "SHOT CPU took" << eltimer.nsecsElapsed()/1000 << "us";
}

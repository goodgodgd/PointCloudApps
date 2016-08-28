#include "pcldescriptors.h"

PclDescriptors::PclDescriptors()
    : useGpu(0)
    , indicesptr (new std::vector<int>)
{
}

void PclDescriptors::ComputeWholeDescriptors(SharedData* shdDat, const int gpuUse, const float descriptorRadius, const int maxNeighbors)
{
    qDebug() << "---------- PCL descriptors ----------";
    pclConverter.ConvertToPCLPointCloud(shdDat);
    useGpu = gpuUse;
    if(useGpu!=0)
        pclConverter.ConvertToPCLPointVector(shdDat);
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();

    eltimer.start();
    if(useGpu & GpuSel::SPIN)
        spin_cpu.EstimateSpinImage(pclConverter.GetPointCloud(), pclConverter.GetNormalCloud(), nullityMap, descriptorRadius);
    else
        spin_gpu.EstimateSpinImage(pclConverter.pclPoints, pclConverter.pclNormals, nullityMap, descriptorRadius, maxNeighbors);
    qDebug() << "SpinImage took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    if(useGpu & GpuSel::FPFH)
        fpfh_cpu.EstimateFpfh(pclConverter.GetPointCloud(), pclConverter.GetNormalCloud(), nullityMap, descriptorRadius);
    else
        fpfh_gpu.EstimateFpfh(pclConverter.pclPoints, pclConverter.pclNormals, nullityMap, descriptorRadius, maxNeighbors);
    qDebug() << "FPFH took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    shot_cpu.EstimateShot(pclConverter.GetPointCloud(), pclConverter.GetNormalCloud(), nullityMap, descriptorRadius);
    qDebug() << "SHOT took" << eltimer.nsecsElapsed()/1000 << "us";

//    eltimer.start();
//    narf_cpu.EstimateNarf(pclConverter.GetPointCloud(), pclConverter.pclNormalCloud, nullityMap);
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

    ComputeIndexedDescriptors(shdDat, false, descriptorRadius, indicesptr);
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
    ComputeIndexedDescriptors(shdDat, false, descriptorRadius, indicesptr);
}

void PclDescriptors::ComputeIndexedDescriptors(SharedData* shdDat, const int gpuUse, const float descriptorRadius
                                               , boost::shared_ptr<std::vector<int>> indicesptr)
{
    const bool bFilter = false;
    pclConverter.ConvertToPCLPointCloud(shdDat, bFilter);
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();
    useGpu = 0;


    eltimer.start();
    fpfh_cpu.EstimateFpfh(pclConverter.GetPointCloud(), pclConverter.GetNormalCloud(), nullityMap, descriptorRadius, indicesptr);
    qDebug() << "FPFH CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    spin_cpu.EstimateSpinImage(pclConverter.GetPointCloud(), pclConverter.GetNormalCloud(), nullityMap, descriptorRadius, indicesptr);
    qDebug() << "SpinImage CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    shot_cpu.EstimateShot(pclConverter.GetPointCloud(), pclConverter.GetNormalCloud(), nullityMap, descriptorRadius, indicesptr);
    qDebug() << "SHOT CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    trisi_cpu.EstimateTrisi(pclConverter.GetPointCloud(), pclConverter.GetThreeAxesCloud(), nullityMap, descriptorRadius, indicesptr);
    qDebug() << "TriSI CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    return;

    for(int di=0; di<smin(10, spin_cpu.descriptors->size()); ++di)
    {
        float dist[3];
        for(int ai=0; ai<3; ++ai)
        {
            dist[ai]=0;
            for(int hi=0; hi<SpinImageType::descriptorSize(); ++hi)
                dist[ai] += fabsf(spin_cpu.descriptors->at(di).histogram[hi] - trisi_cpu.descriptors->at(di).histogram[ai*SPIN_SIZE+hi]);
        }
        qDebug() << "spin dist" << dist[0] << dist[1] << dist[2];
    }
}

#include "clustererbydbrect.h"

ClustererByDbRect::ClustererByDbRect()
{
}

void ClustererByDbRect::FindDbObjects(SharedData* shdDat, const vecAnnot& annots)
{
    qDebug() << "annots length" << annots.size();
    for(const auto& anno : annots)
    {
        qDebug() << anno.category << anno.instanceID << anno.imrect;
    }
}

const cl_int* ClustererByDbRect::GetObjectMap()
{

}

const vecSegment* ClustererByDbRect::GetObjects()
{

}

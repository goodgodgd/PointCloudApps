#include "inclusiontree.h"

MergeableGraph::MergeableGraph(const vecSegment& planes_, ConvexDeterminerType convexDeterminer_)
    : planes(planes_), convexityDeterminer(convexDeterminer_)
{
}

void MergeableGraph::AddConnection(int index1, int index2)
{
//    if(planes[index1].id==154 || planes[index2].id==154)
//        qDebug() << "add" << index1 << planes[index1].id << index2 << planes[index2].id;
    connectionList.emplace_back(index1, index2);
}

vecInts MergeableGraph::ExtractMergeList(const int baseIndex)
{
    vecInts mergeList;
    vecInts mergePath;
    CollectPlanesInSameObject(baseIndex, mergeList, mergePath);
    if(mergeList.size() == 1)
        mergeList.clear();
//    else if(mergeList.size() > 1)
//    {
//        QDebug dbg = qDebug();
//        dbg << "merge" << baseIndex << planes[baseIndex].id << planes[baseIndex].numpt;
//        for(int idx : mergeList)
//            dbg << "," << idx << planes[idx].id;
//    }

    return mergeList;
}

void MergeableGraph::CollectPlanesInSameObject(const int nodeIndex, vecInts& planePool, vecInts mergePath)
{
    if(planes[nodeIndex].id==Segment::SEG_INVALID)
        return;
    if(ExistInIndexList(mergePath, nodeIndex))
        return;

    if(ExistInIndexList(planePool, nodeIndex))
        ;
    else if(IsIncludable(mergePath, nodeIndex))
        planePool.push_back(nodeIndex);
    else
        return;

    mergePath.push_back(nodeIndex);
//    {
//        QDebug dbg = qDebug();
//        dbg << "addpath src" << nodeIndex << "path";
//        for(const int item : mergePath)
//            dbg << item;
//    }


    for(PairOfInts idcPair : connectionList)
    {
        if(idcPair.first==nodeIndex)
            CollectPlanesInSameObject(idcPair.second, planePool, mergePath);
        else if(idcPair.second==nodeIndex)
            CollectPlanesInSameObject(idcPair.first, planePool, mergePath);
    }
}

bool MergeableGraph::IsIncludable(const vecInts& compareList, const int srcIndex)
{
    if(compareList.empty())
        return true;
//    if(ExistInIndexList(compareList, srcIndex))
//        return false;

    for(const int other : compareList)
    {
        for(PairOfInts idcPair : connectionList)
        {
            if((idcPair.first==srcIndex && idcPair.second==other) || idcPair.first==other && idcPair.second==srcIndex)
                return true;
        }
        if(convexityDeterminer(planes[srcIndex], planes[other]))
            return false;
    }
    return true;
}

bool MergeableGraph::ExistInIndexList(const vecInts& list, const int query)
{
    if(list.empty())
        return false;
    for(const int item : list)
        if(item==query)
            return true;
    return false;
}

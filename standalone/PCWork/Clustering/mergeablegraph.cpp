#include "mergeablegraph.h"

MergeableGraph::MergeableGraph(const vecSegment& planes_, ConvexDeterminerType convexDeterminer_)
    : planes(planes_), convexityDeterminer(convexDeterminer_)
{
}

void MergeableGraph::AddConnection(int index1, int index2)
{
    connectionList.emplace_back(index1, index2);
}

vecInts MergeableGraph::ExtractMergeList(const int baseIndex)
{
    vecInts mergeList;
    vecInts mergePath;
    CollectPlanesInSameObject(baseIndex, mergeList, mergePath);
    if(mergeList.size() == 1)
        mergeList.clear();

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

    if(mergePath.size()>=MAX_PATH_LEN)
        return;

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

    for(const int other : compareList)
    {
        for(PairOfInts idcPair : connectionList)
        {
            if((idcPair.first==srcIndex && idcPair.second==other) || (idcPair.first==other && idcPair.second==srcIndex))
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

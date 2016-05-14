#include "inclusiontree.h"

InclusionTree::InclusionTree(const vecSegment& planes_, ConvexDeterminerType convexDeterminer_)
    : planes(planes_), convexityDeterminer(convexDeterminer_)
{
    InitializeTree();
}

void InclusionTree::InitializeTree()
{
    vecInts emptyList;
    tree.resize(planes.size(), emptyList);
}

void InclusionTree::AddRelation(int includerIndex, int joinIndex)
{
    tree[includerIndex].push_back(joinIndex);
}

vecInts InclusionTree::ExtractMergeList(const int baseIndex)
{
    vecInts mergeList;
    mergeList.push_back(baseIndex);
    CollectPlanesInSameObject(planes[baseIndex], baseIndex, mergeList);
    if(mergeList.size() > 1)
    {
        QDebug dbg = qDebug();
        dbg << "merge" << baseIndex << planes[baseIndex].id << planes[baseIndex].numpt;
        for(int idx : mergeList)
            dbg << "," << idx << planes[idx].id;
    }
    return mergeList;
}

void InclusionTree::CollectPlanesInSameObject(const Segment& basePlane, const int nodeIndex, vecInts& planeList)
{
    if(tree[nodeIndex].empty())
        return;
    for(const int childIndex : tree[nodeIndex])
    {
        if(IsIncludable(childIndex, planeList))
        {
            planeList.push_back(childIndex);
            CollectPlanesInSameObject(basePlane, childIndex, planeList);
        }
    }
}

bool InclusionTree::IsIncludable(const int srcIndex, const vecInts& compareList)
{
    for(const int other : compareList)
        if(other==srcIndex)
            return false;

    for(const int other : compareList)
    {
        if(convexityDeterminer(planes[srcIndex], planes[other]))
            return false;
    }
    return true;
}

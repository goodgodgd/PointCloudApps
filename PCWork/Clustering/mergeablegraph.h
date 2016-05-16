#ifndef MERGEABLEGRAPH_H
#define MERGEABLEGRAPH_H

#include <functional>
#include <QDebug>
#include "Share/shared_types.h"
#include "segment.h"

class MergeableGraph
{
public:
    MergeableGraph(const vecSegment& planes_, ConvexDeterminerType convexDeterminer_);
    void InitializeTree();
    void AddConnection(int index1, int index2);
    vecInts ExtractMergeList(const int baseIndex);

private:
    void CollectPlanesInSameObject(const int nodeIndex, vecInts& planePool, vecInts mergePath);
    bool IsIncludable(const vecInts& compareList, const int srcIndex);
    bool ExistInIndexList(const vecInts& list, const int query);

    vecPairOfInts connectionList;
    const vecSegment& planes;
    ConvexDeterminerType convexityDeterminer;
};

#endif // MERGEABLEGRAPH_H

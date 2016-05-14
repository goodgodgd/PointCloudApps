#ifndef INCLUSIONTREE_H
#define INCLUSIONTREE_H

#include <functional>
#include <QDebug>
#include "Share/shared_types.h"
#include "segment.h"

class InclusionTree
{
public:
    InclusionTree(const vecSegment& planes_, ConvexDeterminerType convexDeterminer_);
    void InitializeTree();
    void AddRelation(int includerIndex, int joinIndex);
    vecInts ExtractMergeList(const int baseIndex);
    int GetSize() { return tree.size(); }

private:
    void CollectPlanesInSameObject(const Segment& basePlane, const int nodeIndex, vecInts& planeList);
    bool IsIncludable(const int srcIndex, const vecInts& compareList);

    vecOfVecInts tree;
    const vecSegment& planes;
    ConvexDeterminerType convexityDeterminer;
};

#endif // INCLUSIONTREE_H

#include "clustererbydbrect.h"
#define IOU_Threshold 0.90
#define numpt_Threshold 0.80


ClustererByDbRect::ClustererByDbRect()
{
}

void ClustererByDbRect::FindDbObjects(SharedData* shdDat, const vecAnnot& annots)
{

    //qDebug() << "annots length" << annots.size();
    objectArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    objectMap = objectArray.GetArrayPtr();
    memcpy(objectMap, shdDat->ConstObjectMap(), objectArray.ByteSize());
    objects = *(shdDat->ConstObjects());

    if(annots.size() != 0){ //if annotation exist, start clustering
        for(const auto& anno : annots)
        {
            vecfixList fixList;
            first = true; //we clustering objects to one ID(first object's ID), so I set a flag 'first'.
            oneid = -1;
            index = 0;
            //qDebug() << anno.category << anno.instanceID << anno.imrect;
            for(auto& seg : objects){
                if(DoRectsInclude(anno.imrect, seg.rect)){
                    MergeIncludeRect(seg);
                }
                else if(DoRectsOverlap(anno.imrect, seg.rect)){
                    MergeOverlapRect(anno,seg,fixList);
                }
                index++;

            }

            if(oneid != -1){
                FixObjectMap(fixList);
            }
        }
    }
}

const cl_int* ClustererByDbRect::GetObjectMap()
{
    return objectMap;
}

const vecSegment* ClustererByDbRect::GetObjects()
{
    return &objects;

}
bool ClustererByDbRect::DoRectsInclude(const ImRect& outerRect, const ImRect& innerRect)
{
    if(outerRect.xl <= innerRect.xl && innerRect.xh <= outerRect.xh && outerRect.yl <= innerRect.yl && innerRect.yh <= outerRect.yh)
        return true;
    return false;
}
void ClustererByDbRect::MergeIncludeRect(Segment& seg)
{
    //qDebug() << "Rects include" << seg.id << seg.rect;
    if(!first){

        for(int x=seg.rect.xl;x<=seg.rect.xh;x++){
            for(int y=seg.rect.yl;y<=seg.rect.yh;y++){
                if(objectMap[IMGIDX(y,x)] == seg.id){
                    objectMap[IMGIDX(y,x)] = oneid;
                }
            }
        }
        AbsorbPlane(objects[baseIndex], seg);
    }
    else{
        first = false;
        oneid = seg.id;
        baseIndex = index;
    }
}

bool ClustererByDbRect::DoRectsOverlap(const ImRect& firstRect, const ImRect& secondRect)
{
    if(firstRect.xh <= secondRect.xl || secondRect.xh <= firstRect.xl || firstRect.yh <= secondRect.yl || secondRect.yh <= firstRect.yl)
        return false;
    return true;
}

void ClustererByDbRect::MergeOverlapRect(const Annotation& anno, Segment& seg, vecfixList& fixList)
{
    ImRect intersectRect = OverlappingRect(anno.imrect,seg.rect);
    if(GetRectsIOU(intersectRect, seg.rect)>IOU_Threshold){
        //qDebug() << "Rects overlapped over IOU_Threshold" << seg.id << seg.rect;
        fixList.emplace_back(seg.id,intersectRect.xl,intersectRect.xh,intersectRect.yl,intersectRect.yh);
    }
    else{
        int numpt = 0;
        for(int y=intersectRect.yl;y<=intersectRect.yh;y++){
            for(int x=intersectRect.xl;x<=intersectRect.xh;x++){
                if(objectMap[IMGIDX(y,x)] == seg.id) numpt++;

            }
        }
        if(float(numpt)/seg.numpt > numpt_Threshold){
            //qDebug() << "Rects overlapped over numpt_Threshold" << seg.id << seg.rect;
            fixList.emplace_back(seg.id,intersectRect.xl,intersectRect.xh,intersectRect.yl,intersectRect.yh);
            AbsorbPlanePart(objects[baseIndex], seg, intersectRect, numpt);
        }
    }
}

ImRect ClustererByDbRect::OverlappingRect(const ImRect& firstRect, const ImRect& secondRect)
{
    ImRect outRect;
    outRect.xl = smax(firstRect.xl, secondRect.xl);
    outRect.xh = smin(firstRect.xh, secondRect.xh);
    outRect.yl = smax(firstRect.yl, secondRect.yl);
    outRect.yh = smin(firstRect.yh, secondRect.yh);

    outRect.xl = smax(outRect.xl-1, 0);
    outRect.xh = smin(outRect.xh+1, IMAGE_WIDTH-1);
    outRect.yl = smax(outRect.yl-1, 0);
    outRect.yh = smin(outRect.yh+1, IMAGE_HEIGHT-1);
    return outRect;
}


float ClustererByDbRect::GetRectsIOU(const ImRect& intersectRect, const ImRect& unionRect)
{
    float IOU = ((intersectRect.xh-intersectRect.xl)*(intersectRect.yh-intersectRect.yl))/((unionRect.xh-unionRect.xl)*(unionRect.yh-unionRect.yl));
    return IOU;
}

void ClustererByDbRect::AbsorbPlane(Segment& basePlane, Segment& mergedPlane)
{
    if(basePlane.id == mergedPlane.id)
        return;
    for(int y=mergedPlane.rect.yl; y<=mergedPlane.rect.yh; y++)
    {
        for(int x=mergedPlane.rect.xl; x<=mergedPlane.rect.xh; x++)
        {
            if(objectMap[IMGIDX(y,x)]==mergedPlane.id)
                objectMap[IMGIDX(y,x)] = basePlane.id;
        }
    }
    mergedPlane.id = Segment::SEG_INVALID;
    basePlane.numpt += mergedPlane.numpt;
    basePlane.rect.ExpandRange(mergedPlane.rect);
}

void ClustererByDbRect::AbsorbPlanePart(Segment& basePlane, Segment& mergedPartPlane, ImRect& intersectRect, int numIntersectpt)
{
    if(basePlane.id == mergedPartPlane.id)
        return;
    for(int y=intersectRect.yl; y<=intersectRect.yh; y++)
    {
        for(int x=intersectRect.xl; x<=intersectRect.xh; x++)
        {
            if(objectMap[IMGIDX(y,x)]==mergedPartPlane.id)
                objectMap[IMGIDX(y,x)] = basePlane.id;
        }
    }
    basePlane.numpt += numIntersectpt;
    mergedPartPlane.numpt -= numIntersectpt;
    basePlane.rect.ExpandRange(intersectRect);
    mergedPartPlane.rect.ReduceRange(intersectRect);
}

void ClustererByDbRect::FixObjectMap(vecfixList& fixList)
{
    for(auto fixObject : fixList){
        //qDebug() << "fixlist" << oneid;
        for(int y=fixObject.rect.yl;y<=fixObject.rect.yh;y++){
            for(int x=fixObject.rect.xl;x<=fixObject.rect.xh;x++){
                if(objectMap[IMGIDX(y,x)] == fixObject.id) objectMap[IMGIDX(y,x)]=oneid;
            }
        }
    }
}

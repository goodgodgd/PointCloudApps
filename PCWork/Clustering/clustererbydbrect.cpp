#include "clustererbydbrect.h"
#define IOU_Threshold 0.9



ClustererByDbRect::ClustererByDbRect()
{
}

void ClustererByDbRect::FindDbObjects(SharedData* shdDat, const vecAnnot& annots)
{

    qDebug() << "annots length" << annots.size();
    SetObjects(shdDat->ConstObjects());
    objectArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    objectMap = objectArray.GetArrayPtr();
    memcpy(objectMap, shdDat->ConstObjectMap(), objectArray.ByteSize());
    //SetObjectMap(shdDat->ConstObjectMap());
    int n = objects->size();

    if(annots.size() != 0){ //if annotation exist, start clustering
        for(const auto& anno : annots)
        {
            bool first = true; //we clustering objects to one ID(first object's ID), so I set a flag 'first'.
            int oneid = -1;
            vecfixList fixList;
            for(const Segment& seg : *objects){
                if(DoRectsInclude(anno.imrect, seg.rect)){
                     qDebug() << "Rects include" << seg.id;
                     if(!first){
                       for(int x=seg.rect.xl;x<=seg.rect.xh;x++){
                           for(int y=seg.rect.yl;y<=seg.rect.yh;y++){
                               if(objectMap[IMGIDX(y,x)] == seg.id){
                                   objectMap[IMGIDX(y,x)] = oneid;
                               }
                           }
                       }
                   }
                   else{
                       first = false;
                       oneid = seg.id;
                   }
                }
                else if(DoRectsOverlap(anno.imrect, seg.rect)){

                   ImRect intersectRect = OverlappingRect(anno.imrect, seg.rect);
                   qDebug() << anno.category << GetRectsIOU(intersectRect, seg.rect);
					if(GetRectsIOU(intersectRect, seg.rect)>IOU_Threshold){
                        fixList.emplace_back(seg.id,intersectRect.xl,intersectRect.xh,intersectRect.yl,intersectRect.yh);
					}
                }
            }
            qDebug() << anno.category << anno.instanceID << anno.imrect;
            if(oneid != -1){
                for(auto fixObject : fixList){
                    for(int y=fixObject.rect.yl;y<=fixObject.rect.yh;y++){
                            for(int x=fixObject.rect.xl;x<=fixObject.rect.xh;x++){
                            if(objectMap[IMGIDX(y,x)] == fixObject.id) objectMap[IMGIDX(y,x)]=oneid;
                        }
                    }
                }
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
	return objects;

}
bool ClustererByDbRect::DoRectsInclude(const ImRect& outerRect, const ImRect& innerRect)
{
    if(outerRect.xl <= innerRect.xl && innerRect.xh <= outerRect.xh && outerRect.yl <= innerRect.yl && innerRect.yh <= outerRect.yh)
        return true;
    return false;
}
bool ClustererByDbRect::DoRectsOverlap(const ImRect& firstRect, const ImRect& secondRect)
{
    if(firstRect.xh <= secondRect.xl || secondRect.xh <= firstRect.xl || firstRect.yh <= secondRect.yl || secondRect.yh <= firstRect.yl)
        return false;
    return true;
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


float ClustererByDbRect::GetRectsIOU(const ImRect& intersectRect, const ImRect& innerRect)
{
    float IOU = (float)((intersectRect.xh-intersectRect.xl)*(intersectRect.yh-intersectRect.yl))/(float)((innerRect.xh-innerRect.xl)*(innerRect.yh-innerRect.yl));
    return IOU;
}

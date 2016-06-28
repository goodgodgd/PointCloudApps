#include "clustererbydbrect.h"

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


    if(annots.size() != 0){
        for(const auto& anno : annots)
        {
           //std::vector<int> idlist;
           bool first = true;
           int oneid;
           for(const Segment& seg : *objects){

               if(DoRectsEqual(anno.imrect, seg.rect)){
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
            }
            qDebug() << anno.category << anno.instanceID << anno.imrect;

        }

    }
}

const cl_int* ClustererByDbRect::GetObjectMap()
{
    return objectMap;
}

const vecSegment* ClustererByDbRect::GetObjects()
{

}
bool ClustererByDbRect::DoRectsEqual(const ImRect& firstRect, const ImRect& secondRect)
{
    if(firstRect.xl <= secondRect.xl && secondRect.xh <= firstRect.xh && firstRect.yl <= secondRect.yl && secondRect.yh <= firstRect.yh)
        return true;
    return false;
}


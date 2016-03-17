//seongwon

#include "planeextractor.h"

PlaneExtractor::PlaneExtractor()
{
    planemap = new int[IMAGE_HEIGHT*IMAGE_WIDTH];
    normalCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];
    planes = new vector<Plane>;
}

PlaneExtractor::~PlaneExtractor()
{
    delete[] planemap;
    delete[] normalCloud;
    delete[] planes;
}

void PlaneExtractor::SetInputs(cl_float4* srcNormalCloud){

    memcpy(normalCloud, srcNormalCloud, IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(cl_float4));
}

void PlaneExtractor::Work(){

    ExtractPlanes(normalCloud, planes, &planeNum);

}

void PlaneExtractor::ExtractPlanes(cl_float4* normalCloud, vector<Plane>*& planes, int* planeNum){
    int x,y,n;
    int tempx,tempy;
    int countPixel=0;
    int planeID = 0;//initial planeID

    count = 0;
    QFile *file = new QFile;
    QString filename = "planemap.txt";
    file->setFileName(filename);
    file->open(QIODevice::WriteOnly);
    QTextStream out(file);


    smalls_num=0;
    for(n=0;n<IMAGE_WIDTH*IMAGE_HEIGHT;n++){
        planemap[n]=-1;
        normalCloud[n] = clNormalize(normalCloud[n]);
    }

    for(y=0; y<IMAGE_HEIGHT; y++){
       for(x=0; x<IMAGE_WIDTH; x++) {
            if(planemap[xy2num(x,y)]<0){
                countPixel = 0;
                CompareNormal(x, y, normalCloud, planeID, &countPixel);
                if(countPixel < 30){
                    smalls[smalls_num]=planeID;
                }
                planeID++;
            }
        }
    }
    for(y=0; y<IMAGE_HEIGHT; y++){
        for(x=0; x<IMAGE_WIDTH; x++) {
            for(n=0; n<smalls_num; n++){
                if(planemap[xy2num(x,y)]==smalls[n]){
                    planemap[xy2num(x,y)]=NotPlane;
                }

            }
        }

    }
    qDebug() << "planeID" <<planeID;

    //qDebug() <<x<<y << xy2num(4,5)<<normalCloud[xy2num(200,230)]<< normalCloud[310+239*320];
    for(y=0; y<IMAGE_HEIGHT ; y++){
        for(x=0; x<IMAGE_WIDTH ; x++){
            out << planemap[xy2num(x,y)] << " ";
        }
        out << "\n";
    }
    file->close();
    *planeNum = planeID;
    qDebug() << count;
}




void PlaneExtractor::CompareNormal(int x, int y, cl_float4* normalCloud, int planeID, int* countPixel){

    if(planemap[xy2num(x,y)]==-1)
    {
        if(isnanf(normalCloud[y*IMAGE_WIDTH+x].x) || isnanf(normalCloud[y*IMAGE_WIDTH+x].y)){
            planemap[xy2num(x,y)]=NotPlane;
        }
        else{
            *countPixel = *countPixel + 1;
            planemap[xy2num(x,y)]=planeID;

            if(x!=0){
                if(clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x-1,y)])>Threshold){
                    CompareNormal(x-1, y, normalCloud, planeID, countPixel);
                }
                else{
                    if(x!=1){
                        if(clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x-2,y)])>Threshold)
                        {
                            CompareNormal(x-1, y, normalCloud, planeID, countPixel);
                            CompareNormal(x-2, y, normalCloud, planeID, countPixel);
                        }
                    }
                   //qDebug() << x << y << "left" <<clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x-1,y)]);
                }
            }
            if(x!=IMAGE_WIDTH-1){
                if(clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x+1,y)])>Threshold){
                    CompareNormal(x+1, y, normalCloud, planeID, countPixel);
                }
                else{
                    if(x!=IMAGE_WIDTH-2){
                        if(clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x+2,y)])>Threshold)
                        {
                            CompareNormal(x+1, y, normalCloud, planeID, countPixel);
                            CompareNormal(x+2, y, normalCloud, planeID, countPixel);
                        }
                    }
                    //qDebug() << x << y << "right" <<clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x+1,y)]);
                }
            }
            if(y!=0){
                if(clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x,y-1)])>Threshold){
                    CompareNormal(x, y-1, normalCloud, planeID, countPixel);
                }
                else{
                    if(y!=1){
                        if(clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x,y-2)])>Threshold)
                        {
                            CompareNormal(x, y-1, normalCloud, planeID, countPixel);
                            CompareNormal(x, y-2, normalCloud, planeID, countPixel);
                        }
                    }
                    //qDebug() << x << y << "down" <<clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x,y-1)]);
                }
            }
            if(y!=IMAGE_HEIGHT-1){
                if(clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x,y+1)])>Threshold){
                    CompareNormal(x, y+1, normalCloud, planeID, countPixel);
                }
                else{
                    if(y!=IMAGE_HEIGHT-2){
                        if(clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x,y+2)])>Threshold)
                        {
                            CompareNormal(x, y+1, normalCloud, planeID, countPixel);
                            CompareNormal(x, y+2, normalCloud, planeID, countPixel);
                        }
                    }
                    //qDebug() << x << y << "up" <<clDot(normalCloud[xy2num(x,y)],normalCloud[xy2num(x,y+1)]);
                }
            }
        }
    }


}

int PlaneExtractor::xy2num(int x,  int y){
    return y*IMAGE_WIDTH+x;
}

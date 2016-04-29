//seongwon

// 조:normal vector difference - depth 례
// 수직 거리 비교
// 한번에 하지말고 평면 객체 따로

#include "planeextractor.h"

PlaneExtractor::PlaneExtractor()
{
    planemap = new int[IMAGE_HEIGHT*IMAGE_WIDTH];
    planes = new vector<Plane>;
}

PlaneExtractor::~PlaneExtractor()
{
    delete[] planemap;
    delete[] planes;
}


void PlaneExtractor::ExtractPlanes(cl_float4* normalCloud)
{
    int x,y,n;
    int countPixel=0;
    int planeID = 0;//initial planeID

    smalls_num=0;
    for(n=0;n<IMAGE_WIDTH*IMAGE_HEIGHT;n++)
    {
        planemap[n]=-1;
        normalCloud[n] = clNormalize(normalCloud[n]);
    }

    for(y=0; y<IMAGE_HEIGHT; y++)
    {
       for(x=0; x<IMAGE_WIDTH; x++)
       {
            if(planemap[IMGIDX(y,x)]==-1)
            {
                countPixel = 0;
                CompareNormal(x, y, normalCloud, planeID, &countPixel);
                if(countPixel < 30){
                    smalls[smalls_num]=planeID;

                }
                planeID++;
            }
        }
    }
    for(y=0; y<IMAGE_HEIGHT; y++)
    {
        for(x=0; x<IMAGE_WIDTH; x++)
        {
            for(n=0; n<smalls_num; n++)
            {
                if(planemap[IMGIDX(y,x)]==smalls[n])
                {
                    planemap[IMGIDX(y,x)]=NotPlane;
                }
            }
        }
    }
    qDebug() << "planeID is" <<planeID;

    planeNum = planeID;
}

void PlaneExtractor::CompareNormal(int x, int y, cl_float4* normalCloud, int planeID, int* countPixel){

    //float Threshold = pointCloud[IMGIDX(y,x)].x;
    static const cl_int2 direction[] = {{-1,0},{1,0},{0,1},{0,-1}};//left, right, up, dwon
    if(planemap[IMGIDX(y,x)]==-1)
    {
        if(isnanf(normalCloud[y*IMAGE_WIDTH+x].x) || isnanf(normalCloud[y*IMAGE_WIDTH+x].y))
        {
            planemap[IMGIDX(y,x)]=NotPlane;
        }
        else{
            *countPixel = *countPixel + 1;
            planemap[IMGIDX(y,x)]=planeID;

            bool move[4] = {1,1,1,1};
            bool move_twice[4] = {1,1,1,1};

            if(x==0) move[0]=0;
            else if(x==1) move_twice[0]=0;

            if(x==IMAGE_WIDTH-1) move[1]=0;
            else if(x==IMAGE_WIDTH-2) move_twice[1]=0;

            if(y==IMAGE_HEIGHT-1) move[2]=0;
            else if(y==IMAGE_HEIGHT-2) move_twice[2]=0;

            if(y==0) move[3]=0;
            else if(y==1) move_twice[3]=0;

            for(int i=0; i<4; i++){
                int x_move1 = x+direction[i].x;
                int y_move1 = y+direction[i].y;
                int x_move2 = x+direction[i].x*2;
                int y_move2 = y+direction[i].y*2;

                if(move[i] && clDot(normalCloud[IMGIDX(y,x)],normalCloud[IMGIDX(y_move1,x_move1)])>Threshold)
                {
                    CompareNormal(x_move1, y_move1, normalCloud, planeID, countPixel);
                }
                else if(move_twice[i] && clDot(normalCloud[IMGIDX(y,x)],normalCloud[IMGIDX(y_move2,x_move2)])>Threshold)
                {
                    CompareNormal(x_move1, y_move1, normalCloud, planeID, countPixel);
                    CompareNormal(x_move2, y_move2, normalCloud, planeID, countPixel);
                }
            }
        }
    }
}

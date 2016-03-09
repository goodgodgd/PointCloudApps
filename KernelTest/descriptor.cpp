#include "descriptor.h"

#define NB_HALF_DIM     10
#define NB_DIM          (NB_HALF_DIM*2+1)
#define NUM_PTS         (NB_DIM*NB_DIM)
#define NB_IDX(r,c)     (r*NB_DIM+c)


void TestDescriptor()
{
    cl_float4 trueDesc = (cl_float4){0.3f, 0.2f, 0, 0};
    cl_float4 neighborCloud[NUM_PTS];
    cl_float4 translation = (cl_float4){1, 2, 3, 0};
    cl_float4 rotation = (cl_float4){0.5f, 0, 0, 0};
    cl_float4 normalvt = (cl_float4){0.f, 0.f, 1.f, 0};
    cl_float4 descriptor;

    qDebug() << "TestDescriptor - true descriptor:" << trueDesc;
    int num_pts = CreatePointCloud(trueDesc, neighborCloud);
    TransformPointCloud(rotation, translation, neighborCloud, num_pts, normalvt);
    ComputeDescriptor(translation, normalvt, neighborCloud, num_pts, descriptor);
}

int CreatePointCloud(cl_float4 trueDesc, cl_float4* pointCloud)
{
    const float itv = 0.02f;
    cl_float4 point;

    for(int r=0; r<NB_DIM; r++)
    {
        for(int c=0; c<NB_DIM; c++)
        {
            point.x = (r-NB_HALF_DIM)*itv;
            point.y = (c-NB_HALF_DIM)*itv;
            point.z = trueDesc.x*point.x*point.x + trueDesc.y*point.y*point.y;
            pointCloud[NB_IDX(r,c)] = point;
        }
    }
    return NUM_PTS;
}

void TransformPointCloud(cl_float4 rotation, cl_float4 translation, cl_float4* pointCloud, int num_pts, cl_float4& normalvt)
{
    cl_float4 rotmat[3];
    rotmat[0] = cl_float4{1,0,0,0};
    rotmat[1] = cl_float4{0,cosf(rotation.x),-sinf(rotation.x),0};
    rotmat[2] = cl_float4{0,sinf(rotation.x),cosf(rotation.x),0};
    cl_float4 npoint;

    for(int i=0; i<num_pts; i++)
    {
        npoint = pointCloud[i];
        pointCloud[i].x = clDot(rotmat[0], npoint);
        pointCloud[i].y = clDot(rotmat[1], npoint);
        pointCloud[i].z = clDot(rotmat[2], npoint);
        pointCloud[i] = pointCloud[i] + translation;
    }

    npoint = normalvt;
    normalvt.x = clDot(rotmat[0], npoint);
    normalvt.y = clDot(rotmat[1], npoint);
    normalvt.z = clDot(rotmat[2], npoint);
}

void ComputeDescriptor(cl_float4 ctpoint, cl_float4 ctnormal, cl_float4* neighborCloud, int num_pts, DescType& descriptor)
{
//    int x = IMAGE_WIDTH/2;
//    int y = IMAGE_HEIGHT/2;
    int x = 0;
    int y = 0;
    int ptpos = (y*IMAGE_WIDTH + x);
    int nbbegin = ptpos*NEIGHBORS_PER_POINT;

    // matrix for linear equation, solution vector
    float linEq[L_DIM*L_WIDTH];
    float ysol[L_DIM];
    for(int i=0; i<L_DIM*L_WIDTH; i++)
        linEq[i] = 0;
    for(int i=0; i<L_DIM; i++)
        ysol[i] = 0;

    // set linear equation for matrix A
    // set F'*F part
    SetUpperLeft(neighborCloud, ctpoint, nbbegin, num_pts, linEq);
    // set G parts
    SetUpperRight(ctnormal, linEq);
    SetLowerLeft(ctnormal, linEq);
    // set b in Ax=b
    SetRightVector(neighborCloud, ctpoint, ctnormal, nbbegin, num_pts, linEq);
//    PrintMatrix(L_DIM, L_WIDTH, linEq, "Linear eq");

    SolveLinearEq(L_DIM, linEq, ysol);
//    PrintVector(NUM_VAR, ysol, "\nvectorized A");

    // compute shape descriptor by using eigen decomposition
    float egval[PT_DIM];
    float egvec[PT_DIM*PT_DIM];
    GetDescriptorByEigenDecomp(ysol, egval, egvec);
    PrintVector(PT_DIM, egval, "computed descriptor");
//    PrintMatrix(PT_DIM, PT_DIM, egvec, "eigenvectors of A");
}

void SetUpperLeft(cl_float4* neighborCloud, cl_float4 centerpt, int offset, int num_pts, float* L)
{
    cl_float4 diff;
    cl_float8 rowOfF;
    for(int i=offset; i<offset+num_pts; i++)
    {
        diff = neighborCloud[i] - centerpt;

        // set each row of F
        rowOfF.s[0] = diff.x*diff.x;
        rowOfF.s[1] = diff.y*diff.y;
        rowOfF.s[2] = diff.z*diff.z;
        rowOfF.s[3] = 2.f*diff.x*diff.y;
        rowOfF.s[4] = 2.f*diff.y*diff.z;
        rowOfF.s[5] = 2.f*diff.z*diff.x;

        // compute F'*F
        for(int r=0; r<NUM_VAR; r++)
            for(int c=0; c<NUM_VAR; c++)
                L[L_INDEX(r,c)] += rowOfF.s[r] * rowOfF.s[c];
    }
}

void SetUpperRight(cl_float4 normal, float* L)
{
    int bgx = 6;
    int bgy = 0;

    L[L_INDEX(bgy+0,bgx+0)] = normal.x;
    L[L_INDEX(bgy+3,bgx+0)] = normal.y;
    L[L_INDEX(bgy+5,bgx+0)] = normal.z;

    L[L_INDEX(bgy+3,bgx+1)] = normal.x;
    L[L_INDEX(bgy+1,bgx+1)] = normal.y;
    L[L_INDEX(bgy+4,bgx+1)] = normal.z;

    L[L_INDEX(bgy+5,bgx+2)] = normal.x;
    L[L_INDEX(bgy+4,bgx+2)] = normal.y;
    L[L_INDEX(bgy+2,bgx+2)] = normal.z;
}

void SetLowerLeft(cl_float4 normal, float* L)
{
    int bgx = 0;
    int bgy = 6;

    L[L_INDEX(bgy+0,bgx+0)] = normal.x;
    L[L_INDEX(bgy+0,bgx+3)] = normal.y;
    L[L_INDEX(bgy+0,bgx+5)] = normal.z;

    L[L_INDEX(bgy+1,bgx+3)] = normal.x;
    L[L_INDEX(bgy+1,bgx+1)] = normal.y;
    L[L_INDEX(bgy+1,bgx+4)] = normal.z;

    L[L_INDEX(bgy+2,bgx+5)] = normal.x;
    L[L_INDEX(bgy+2,bgx+4)] = normal.y;
    L[L_INDEX(bgy+2,bgx+2)] = normal.z;
}

void SetRightVector(cl_float4* neighborCloud, cl_float4 ctpoint, cl_float4 ctnormal, int offset, int num_pts, float* L)
{
    cl_float4 diff;
    cl_float4 fx[NUM_VAR];
    cl_float8 rowOfF;
    for(int i=0; i<NUM_VAR; i++)
        fx[i] = (cl_float4){0,0,0,0};

    for(int i=offset; i<offset+num_pts; i++)
    {
        diff = neighborCloud[i] - ctpoint;
        // set each row of F
        rowOfF.s[0] = diff.x*diff.x;
        rowOfF.s[1] = diff.y*diff.y;
        rowOfF.s[2] = diff.z*diff.z;
        rowOfF.s[3] = 2.f*diff.x*diff.y;
        rowOfF.s[4] = 2.f*diff.y*diff.z;
        rowOfF.s[5] = 2.f*diff.z*diff.x;

        for(int r=0; r<NUM_VAR; r++)
            for(int c=0; c<PT_DIM; c++)
                fx[r].s[c] += rowOfF.s[r] * diff.s[c];
    }

    for(int i=0; i<NUM_VAR; i++)
        L[L_INDEX(i,L_DIM)] = clDot(fx[i], ctnormal);
}

void GetDescriptorByEigenDecomp(float Avec[NUM_VAR], float egval[PT_DIM], float egvec[PT_DIM*PT_DIM])
{
    // convert solution to 3x3 matrix
    float Amat[PT_DIM][PT_DIM];
    Amat[0][0] = Avec[0];
    Amat[1][1] = Avec[1];
    Amat[2][2] = Avec[2];
    Amat[0][1] = Amat[1][0] = Avec[3];
    Amat[1][2] = Amat[2][1] = Avec[4];
    Amat[0][2] = Amat[2][0] = Avec[5];

    // eigen decomposition using Eigen3
    Eigen::Matrix3f Emat;
    for(int i=0; i<PT_DIM; i++)
        for(int k=0; k<PT_DIM; k++)
            Emat(i,k) = Amat[i][k];

    Eigen::EigenSolver<Eigen::Matrix3f> eigensolver(Emat);
    Eigen::Vector3f Eval = eigensolver.eigenvalues().real();
    Eigen::Matrix3f Evec = eigensolver.eigenvectors().real();

    for(int i=0; i<PT_DIM; i++)
    {
        egval[i] = Eval(i);
        for(int k=0; k<PT_DIM; k++)
            egvec[i*PT_DIM+k] = Evec(i,k);
    }

    // sort eigenvalues and eigenvectors w.r.t eigenvalues
    int first=0, second=1;
    // swap first largest eigenvalue with eigenvalue[0]
    if(egval[0] <= egval[2] && egval[1] <= egval[2])
        first = 2;
    else if(egval[0] <= egval[1] && egval[2] <= egval[1])
        first = 1;
    SwapEigen(egval, egvec, first, 0);

    // swap second largest eigenvalue with eigenvalue[1]
    if(egval[1] < egval[2])
        second = 2;
    SwapEigen(egval, egvec, second, 1);
}

void SwapEigen(float egval[PT_DIM], float egvec[PT_DIM*PT_DIM], int src, int dst)
{
    if(src==dst)
        return;
    float tmp;
    // swap eigenvalue
    tmp = egval[dst];
    egval[dst] = egval[src];
    egval[src] = tmp;
    // swap eigenvector
    for(int i=0; i<PT_DIM; i++)
    {
        tmp = egvec[i*PT_DIM+dst];
        egvec[i*PT_DIM+dst] = egvec[i*PT_DIM+src];
        egvec[i*PT_DIM+src] = tmp;
    }
}

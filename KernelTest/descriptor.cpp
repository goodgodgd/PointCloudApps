#include "descriptor.h"

void TestDescriptor()
{
#define NB_HALF_DIM     10
#define NB_DIM          (NB_HALF_DIM*2+1)
#define NUM_PTS         (NB_DIM*NB_DIM+1)
#define NB_IDX(r,c)     (r*NB_DIM+c+1)

    const float itv = 0.02f;
    cl_float4 neighborCloud[NUM_PTS];
    cl_float4 normalCloud = (cl_float4){0.f, 0.f, 1.f, 0};
    cl_float4 descCloud;
    cl_float4 trueDesc = (cl_float4){0.3f, 0.1f, 0, 0};
    cl_float4 point;
    cl_float4 ctposit = (cl_float4){0, 0, 0, 0};

    neighborCloud[0] = ctposit;
    neighborCloud[0].w = NUM_PTS;
    for(int r=0; r<NB_DIM; r++)
    {
        for(int c=0; c<NB_DIM; c++)
        {
            point.x = (r-NB_HALF_DIM)*itv;
            point.y = (c-NB_HALF_DIM)*itv;
            point.z = trueDesc.x*point.x*point.x + trueDesc.y*point.y*point.y;
            neighborCloud[NB_IDX(r,c)] = point + ctposit;
        }
    }

    qDebug() << NUM_PTS << " point created" << endl;
//    for(int i=0; i<20; i++)
//        qDebug() << qSetRealNumberPrecision(5) << "pt" << i << neighborCloud[i];
//    return;


    ComputeDescriptor(neighborCloud, &normalCloud, &descCloud);
}

void ComputeDescriptor(cl_float4* neighborCloud, cl_float4* normalCloud, DescType* descCloud)
{
//    int x = IMAGE_WIDTH/2;
//    int y = IMAGE_HEIGHT/2;
    int x = 0;
    int y = 0;
    int ptpos = (y*IMAGE_WIDTH + x);
    int nbbegin = ptpos*NEIGHBORS_PER_POINT;
    cl_float4 ctpoint = neighborCloud[nbbegin];
    cl_float4 ctnormal = normalCloud[ptpos];
    int num_pts = ctpoint.w;

    // matrix for linear equation, solution vector
    float linEq[L_DIM*L_WIDTH];
    float ysol[L_DIM];
    for(int i=0; i<L_DIM*L_WIDTH; i++)
        linEq[i] = 0;
    for(int i=0; i<L_DIM; i++)
        ysol[i] = 0;

    // set linear equation for matrix A
    // set F'*F part
    SetUpperLeft(neighborCloud, ctpoint, nbbegin+1, num_pts, linEq);
    // set G parts
    SetUpperRight(ctnormal, linEq);
    SetLowerLeft(ctnormal, linEq);
    // set b in Ax=b
    SetRightVector(neighborCloud, ctpoint, ctnormal, nbbegin+1, num_pts, linEq);
    PrintMatrix(L_DIM, L_WIDTH, linEq, "Linear eq");

    SolveLinearEq(L_DIM, linEq, ysol);
//    PrintMatrix(L_DIM, L_WIDTH, linEq, "Gauss elim");
    PrintVector(L_DIM, ysol, "vectorized A");

    // compute shape descriptor by using eigen decomposition
    float egval[PT_DIM];
    float egvec[PT_DIM*PT_DIM];
    GetDescriptorByEigenDecomp(ysol, egval, egvec);
    PrintVector(PT_DIM, egval, "eigenvalues of A");
    PrintMatrix(PT_DIM, PT_DIM, egvec, "eigenvectors of A");
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
        {
            for(int c=0; c<NUM_VAR; c++)
            {
                L[L_INDEX(r,c)] += rowOfF.s[r] * rowOfF.s[c];

                if(isnan(L[L_INDEX(r,c)]) || isinf(L[L_INDEX(r,c)]))
                {
                    cout << "NaN!! diff " << setw(13) << diff.x << " " << diff.y << " " << diff.z << " " << r << " " << c << endl;
                    cout << "NaN!! neig " << setw(13) << neighborCloud[i].x << neighborCloud[i].y << neighborCloud[i].z << endl;
                    cout << "NaN!! cent " << setw(13) << centerpt.x << centerpt.y << centerpt.z << endl;
                }
            }
        }
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
    for(int i=0; i<NUM_VAR; i++)
        fx[i] = (cl_float4){0,0,0,0};

    for(int i=offset; i<offset+num_pts; i++)
    {
        diff = neighborCloud[i] - ctpoint;
        fx[0].x += diff.x * diff.x * diff.x;
        fx[0].y += diff.x * diff.x * diff.y;
        fx[0].z += diff.x * diff.x * diff.z;

        fx[1].x += diff.y * diff.y * diff.x;
        fx[1].y += diff.y * diff.y * diff.y;
        fx[1].z += diff.y * diff.y * diff.z;

        fx[2].x += diff.z * diff.z * diff.x;
        fx[2].y += diff.z * diff.z * diff.y;
        fx[2].z += diff.z * diff.z * diff.z;

        fx[3].x += diff.x * diff.y * diff.x;
        fx[3].y += diff.x * diff.y * diff.y;
        fx[3].z += diff.x * diff.y * diff.z;

        fx[4].x += diff.y * diff.z * diff.x;
        fx[4].y += diff.y * diff.z * diff.y;
        fx[4].z += diff.y * diff.z * diff.z;

        fx[5].x += diff.z * diff.x * diff.x;
        fx[5].y += diff.z * diff.x * diff.y;
        fx[5].z += diff.z * diff.x * diff.z;
    }

    for(int i=0; i<NUM_VAR; i++)
        L[L_INDEX(i,L_DIM)] += clDot(fx[i], ctnormal);
}

void GetDescriptorByEigenDecomp(float Avec[NUM_VAR], float egval[PT_DIM], float egvec[PT_DIM*PT_DIM])
{
    // convert solution to 3x3 matrix
    float Amat[PT_DIM][PT_DIM];
    Amat[0][0] = Avec[0];
    Amat[1][1] = Avec[1];
    Amat[2][2] = Avec[2];
    Amat[0][1] = Amat[1][0] = Avec[3];
    Amat[0][2] = Amat[2][0] = Avec[4];
    Amat[1][2] = Amat[2][1] = Avec[5];

    // eigen decomposition using Eigen3
    Eigen::Matrix3f Emat;
    for(int i=0; i<PT_DIM; i++)
        for(int k=0; k<PT_DIM; k++)
            Emat(i,k) = Amat[i][k];

    Eigen::EigenSolver<Eigen::Matrix3f> eigensolver(Emat);
    Eigen::Vector3f Eval = eigensolver.eigenvalues().real();
    Eigen::Matrix3f Evec = eigensolver.eigenvectors().real();
    cout << "Eigen values" << Eval.transpose() << endl;
    cout << "Eigen vectors" << endl << Evec << endl;

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

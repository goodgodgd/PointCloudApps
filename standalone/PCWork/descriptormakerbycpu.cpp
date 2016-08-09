#include "descriptormakerbycpu.h"
#include <iostream>

DescriptorMakerByCpu::DescriptorMakerByCpu()
{
    descriptorArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    axesArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
}

void DescriptorMakerByCpu::ComputeDescriptors(const cl_float4* pointCloud, const cl_float4* normalCloud
                                              , const cl_int* neighborIndices, const cl_int* numNeighbors, const int maxNeighbs)
{
    DescType* descriptors = descriptorArray.GetArrayPtr();
    memset(descriptors, 0x00, descriptorArray.ByteSize());

    for(int y=0; y<IMAGE_HEIGHT; y++)
        for(int x=0; x<IMAGE_WIDTH; x++)
            ComputeCurvature(IMGIDX(y,x), pointCloud, normalCloud, neighborIndices, numNeighbors, maxNeighbs);
}

void DescriptorMakerByCpu::ComputeCurvature(const int pxidx, const cl_float4* pointCloud, const cl_float4* normalCloud
                                            , const cl_int* neighborIndices, const cl_int* numNeighbors, const int maxNeighbs)
{
    const cl_float4& ctpoint = pointCloud[pxidx];
    //==============================================
    // minus normal
    const cl_float4& ctnormal = -normalCloud[pxidx];
    //==============================================
    const int niOffset = pxidx*maxNeighbs;
    DescType* descriptors = descriptorArray.GetArrayPtr();
    AxesType* axes = axesArray.GetArrayPtr();

    // matrix for linear equation, solution vector
    Eigen::MatrixXf LEA = Eigen::MatrixXf::Zero(L_DIM,L_DIM);
    Eigen::VectorXf LEb = Eigen::VectorXf::Zero(L_DIM);
    Eigen::VectorXf ysol = Eigen::VectorXf::Zero(L_DIM);

    // set linear equation for matrix A
    // set F'*F part
    SetUpperLeft(ctpoint, pointCloud, neighborIndices, niOffset, numNeighbors[pxidx], LEA);
    // set G parts
    SetUpperRight(ctnormal, LEA);
    SetLowerLeft(ctnormal, LEA);
    // set b in Ax=b
    SetRightVector(ctpoint, pointCloud, ctnormal, neighborIndices, niOffset, numNeighbors[pxidx], LEb);
    // solve y
    ysol = LEA.colPivHouseholderQr().solve(LEb);

    // convert solution to 3x3 matrix
    Eigen::Matrix3f Amat(PT_DIM,PT_DIM);
    Amat(0,0) = ysol(0);
    Amat(1,1) = ysol(1);
    Amat(2,2) = ysol(2);
    Amat(0,1) = Amat(1,0) = ysol(3);
    Amat(1,2) = Amat(2,1) = ysol(4);
    Amat(0,2) = Amat(2,0) = ysol(5);

    Eigen::EigenSolver<Eigen::Matrix3f> eigensolver(Amat);
    Eigen::Vector3f eval = eigensolver.eigenvalues().real();
    Eigen::Matrix3f evec = eigensolver.eigenvectors().real();

    SortEigens(eval, evec);

    // rescale descriptor and reverse sign of descriptor
    descriptors[pxidx] = (DescType){eval(0)*EQUATION_SCALE, eval(1)*EQUATION_SCALE, eval(2)*EQUATION_SCALE, 0};
    axes[pxidx] = (AxesType){evec(0,0), evec(1,0), evec(2,0), 0, evec(0,1), evec(1,1), evec(2,1), 0};
}

void DescriptorMakerByCpu::SetUpperLeft(const cl_float4& ctpoint, const cl_float4* pointCloud, const cl_int* neighborIndices
                                        , const int offset, const int num_pts, Eigen::MatrixXf& LEA)
{
    int nbidx;
    cl_float4 diff;
    cl_float8 rowOfF;
    for(int i=offset; i<offset+num_pts; i++)
    {
        nbidx = neighborIndices[i];
        diff = pointCloud[nbidx] - ctpoint;
        diff = diff*EQUATION_SCALE;
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
                LEA(r,c) += rowOfF.s[r] * rowOfF.s[c];
    }
}

void DescriptorMakerByCpu::SetUpperRight(const cl_float4& normal, Eigen::MatrixXf& LEA)
{
    int bgx = NUM_VAR;
    int bgy = 0;

    LEA(bgy+0,bgx+0) = normal.x;
    LEA(bgy+3,bgx+0) = normal.y;
    LEA(bgy+5,bgx+0) = normal.z;

    LEA(bgy+3,bgx+1) = normal.x;
    LEA(bgy+1,bgx+1) = normal.y;
    LEA(bgy+4,bgx+1) = normal.z;

    LEA(bgy+5,bgx+2) = normal.x;
    LEA(bgy+4,bgx+2) = normal.y;
    LEA(bgy+2,bgx+2) = normal.z;
}

void DescriptorMakerByCpu::SetLowerLeft(const cl_float4& normal, Eigen::MatrixXf& LEA)
{
    int bgx = 0;
    int bgy = NUM_VAR;

    LEA(bgy+0,bgx+0) = normal.x;
    LEA(bgy+0,bgx+3) = normal.y;
    LEA(bgy+0,bgx+5) = normal.z;

    LEA(bgy+1,bgx+3) = normal.x;
    LEA(bgy+1,bgx+1) = normal.y;
    LEA(bgy+1,bgx+4) = normal.z;

    LEA(bgy+2,bgx+5) = normal.x;
    LEA(bgy+2,bgx+4) = normal.y;
    LEA(bgy+2,bgx+2) = normal.z;
}

void DescriptorMakerByCpu::SetRightVector(const cl_float4& ctpoint, const cl_float4* pointCloud, const cl_float4& ctnormal
                                          , const cl_int* neighborIndices, const int offset, const int num_pts, Eigen::VectorXf& LEb)
{
    int nbidx;
    cl_float4 diff;
    cl_float4 fx[NUM_VAR];
    cl_float8 rowOfF;
    memset(fx, 0x00, NUM_VAR*sizeof(cl_float4));

    for(int i=offset; i<offset+num_pts; i++)
    {
        nbidx = neighborIndices[i];
        diff = pointCloud[nbidx] - ctpoint;
        diff = diff*EQUATION_SCALE;
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
        LEb(i) = clDot(fx[i], ctnormal);
}

void DescriptorMakerByCpu::SortEigens(Eigen::Vector3f& eval, Eigen::Matrix3f& evec)
{
    // sort eigenvalues and eigenvectors w.r.t eigenvalues
    int first=0, second=1;
    // swap first largest eigenvalue with eigenvalue[0]
    if(fabsf(eval(0)) <= fabsf(eval(2)) && fabsf(eval(1)) <= fabsf(eval(2)))
        first = 2;
    else if(fabsf(eval(0)) <= fabsf(eval(1)) && fabsf(eval(2)) <= fabsf(eval(1)))
        first = 1;
    SwapEigen(eval, evec, first, 0);

    // swap second largest eigenvalue with eigenvalue[1]
    if(fabsf(eval(1)) < fabsf(eval(2)))
    {
        second = 2;
        SwapEigen(eval, evec, second, 1);
    }
}

void DescriptorMakerByCpu::SwapEigen(Eigen::Vector3f& egval, Eigen::Matrix3f& egvec, const int src, const int dst)
{
    if(src==dst)
        return;
    float tmp;
    // swap eigenvalue
    tmp = egval(dst);
    egval(dst) = egval(src);
    egval(src) = tmp;
    // swap eigenvector
    for(int i=0; i<PT_DIM; i++)
    {
        tmp = egvec(i,dst);
        egvec(i,dst) = egvec(i,src);
        egvec(i,src) = tmp;
    }
}

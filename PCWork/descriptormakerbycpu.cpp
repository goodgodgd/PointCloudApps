#include "descriptormakerbycpu.h"
#include <iostream>

int pxidx = 0;

DescriptorMakerByCpu::DescriptorMakerByCpu()
{
    descriptorArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    descriptors = descriptorArray.GetArrayPtr();
}

void DescriptorMakerByCpu::ComputeDescriptors(const cl_float4* pointCloud_, const cl_float4* normalCloud
                                              , const cl_int* neighborIndices, const cl_int* numNeighbors, const int maxNeighbs)
{
    int ptpos;
    memset(descriptors, 0x00, descriptorArray.ByteSize());
    pointCloud = pointCloud_;

    //#pragma omp parallel for
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            ptpos = IMGIDX(y,x);
            pxidx = ptpos;
            if(clIsNull(pointCloud_[ptpos]) || clIsNull(normalCloud[ptpos]))
                continue;
            if(numNeighbors[ptpos] < MIN_NUM_NEIGHBORS)
                continue;

            descriptors[ptpos] = ComputeEachDescriptor(pointCloud[ptpos], normalCloud[ptpos]
                                                       , neighborIndices, ptpos*maxNeighbs, numNeighbors[ptpos]);
        }
    }
}

DescType DescriptorMakerByCpu::ComputeEachDescriptor(const cl_float4& ctpoint, const cl_float4& ctnormal
                                                     , const cl_int* neighborIndices, const int niOffset, const int numNeighbs)
{
    // matrix for linear equation, solution vector
    Eigen::MatrixXf LEA = Eigen::MatrixXf::Zero(L_DIM,L_DIM);
    Eigen::VectorXf LEb = Eigen::VectorXf::Zero(L_DIM);
    Eigen::VectorXf ysol = Eigen::VectorXf::Zero(L_DIM);

    // set linear equation for matrix A
    // set F'*F part
    SetUpperLeft(ctpoint, neighborIndices, niOffset, numNeighbs, LEA);
    // set G parts
    SetUpperRight(ctnormal, LEA);
    SetLowerLeft(ctnormal, LEA);
    // set b in Ax=b
    SetRightVector(ctpoint, ctnormal, neighborIndices, niOffset, numNeighbs, LEb);
    // solve y
    ysol = LEA.colPivHouseholderQr().solve(LEb);


#ifdef DEBUG_ObjectClusterBase
    cl_float4 descriptor = GetDescriptorByEigenDecomp(ysol);
    static int count=0;
    if(fabsf(descriptor.z) > 0.1f && count++ < 20)
    {
        Eigen::MatrixXf LEA1 = Eigen::MatrixXf::Zero(L_DIM,L_WIDTH);
        LEA1.block(0,0,L_DIM,L_DIM) = LEA;
        LEA1.block(0,L_DIM,L_DIM,1) = LEb;
        std::cout << "New: linEq " << count << " " << pxidx << " nn " << numNeighbs << std::endl << LEA1 << std::endl;
        std::cout << "New: ysol " << ysol.transpose() << std::endl;
        std::cout << "New: descrptor " << descriptor.x << " " << descriptor.y << " " << descriptor.z << std::endl;
        DescriptorMakerCpu descMaker;
        descMaker.ComputeEachDescriptor(ctpoint, ctnormal, pointCloud, neighborIndices, niOffset, numNeighbs, true);
    }
    return descriptor;
#else
    // compute shape descriptor by using eigen decomposition
    return GetDescriptorByEigenDecomp(ysol);
#endif
}

void DescriptorMakerByCpu::SetUpperLeft(const cl_float4& ctpoint, const cl_int* neighborIndices, const int offset, const int num_pts
                                        , Eigen::MatrixXf& LEA)
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

void DescriptorMakerByCpu::SetRightVector(const cl_float4& ctpoint, const cl_float4& ctnormal
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

DescType DescriptorMakerByCpu::GetDescriptorByEigenDecomp(const Eigen::VectorXf& Avec)
{
    // convert solution to 3x3 matrix
    Eigen::Matrix3f Amat(PT_DIM,PT_DIM);
    Amat(0,0) = Avec(0);
    Amat(1,1) = Avec(1);
    Amat(2,2) = Avec(2);
    Amat(0,1) = Amat(1,0) = Avec(3);
    Amat(1,2) = Amat(2,1) = Avec(4);
    Amat(0,2) = Amat(2,0) = Avec(5);

    Eigen::EigenSolver<Eigen::Matrix3f> eigensolver(Amat);
    Eigen::Vector3f Eval = eigensolver.eigenvalues().real();
    Eigen::Matrix3f Evec = eigensolver.eigenvectors().real();

//    Eigen::Vector3f EvalOriginal = Eval;

    // sort eigenvalues and eigenvectors w.r.t eigenvalues
    int first=0, second=1;
    // swap first largest eigenvalue with eigenvalue[0]
    if(fabsf(Eval(0)) <= fabsf(Eval(2)) && fabsf(Eval(1)) <= fabsf(Eval(2)))
        first = 2;
    else if(fabsf(Eval(0)) <= fabsf(Eval(1)) && fabsf(Eval(2)) <= fabsf(Eval(1)))
        first = 1;
    SwapEigen(Eval, Evec, first, 0);

    // swap second largest eigenvalue with eigenvalue[1]
    if(fabsf(Eval(1)) < fabsf(Eval(2)))
    {
        second = 2;
        SwapEigen(Eval, Evec, second, 1);
    }

    // rescale descriptor and reverse sign of descriptor
    DescType descriptor;
    for(int i=0; i<PT_DIM; i++)
        descriptor.s[i] = -Eval(i)*EQUATION_SCALE;
    return descriptor;
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

const DescType* DescriptorMakerByCpu::GetDescriptors()
{
    return descriptors;
}

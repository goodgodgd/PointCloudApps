#include "descriptormakercpu.h"

DescriptorMakerCpu::DescriptorMakerCpu()
{
}

void DescriptorMakerCpu::ComputeDescriptors(const cl_float4* pointCloud, const cl_float4* normalCloud
                                    , const cl_int* neighborIndices, const cl_int* numNeighbors, int maxNeighbs
                                    , DescType* descriptors)
{
//    int ptpos = 150*IMAGE_WIDTH + 150;
//    if(IsInvalidPoint(pointCloud[ptpos]) || clIsNull(normalCloud[ptpos]))
//        return;
//    if(numNeighbors[ptpos] < maxNeighbs/2)
//        return;
//    descriptors[ptpos] = ComputeEachDescriptor(pointCloud[ptpos], normalCloud[ptpos]
//                                                   , pointCloud, neighborIndices, ptpos*maxNeighbs, numNeighbors[ptpos], true);
//    return;

    //#pragma omp parallel for
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            int ptpos = y*IMAGE_WIDTH + x;

            if(IsInvalidPoint(pointCloud[ptpos]) || clIsNull(normalCloud[ptpos]))
                continue;
            if(numNeighbors[ptpos] < maxNeighbs/2)
                continue;

            descriptors[ptpos] = ComputeEachDescriptor(pointCloud[ptpos], normalCloud[ptpos]
                                                           , pointCloud, neighborIndices, ptpos*maxNeighbs, numNeighbors[ptpos]);
        }
    }
}

bool DescriptorMakerCpu::IsInvalidPoint(cl_float4 point)
{
    if(point.x < 0.1f)
        return true;
    else
        return false;
}

DescType DescriptorMakerCpu::ComputeEachDescriptor(cl_float4 ctpoint, cl_float4 ctnormal
                                            , const cl_float4* pointCloud, const cl_int* neighborIndices, int niOffset, int numNeighbs
                                            , bool b_print)
{
    // matrix for linear equation, solution vector
    float linEq[DESC_EQUATION_SIZE];
    float ysol[L_DIM];
    for(int i=0; i<DESC_EQUATION_SIZE; i++)
        linEq[i] = 0;
    for(int i=0; i<L_DIM; i++)
        ysol[i] = 0;

    // set linear equation for matrix A
    // set F'*F part
    SetUpperLeft(ctpoint, pointCloud, neighborIndices, niOffset, numNeighbs, linEq);
    // set G parts
    SetUpperRight(ctnormal, linEq);
    SetLowerLeft(ctnormal, linEq);
    // set b in Ax=b
    SetRightVector(ctpoint, ctnormal, pointCloud, neighborIndices, niOffset, numNeighbs, linEq);

    if(b_print)
        PrintMatrix(L_DIM, L_WIDTH, linEq, "Old: linEq");

    SolveLinearEq(L_DIM, linEq, ysol);

    if(b_print)
        PrintVector(L_DIM, ysol, "Old: ysol");

    // compute shape descriptor by using eigen decomposition
    cl_float4 descriptor = GetDescriptorByEigenDecomp(ysol);

    if(b_print)
        std::cout << "Old: descriptor" << descriptor.x << " " << descriptor.y << " " << descriptor.z << std::endl;
//    if(b_print)
//        qDebug() << "descriptor output" << descriptor;
    return descriptor;
}

void DescriptorMakerCpu::SetUpperLeft(cl_float4 ctpoint, const cl_float4* pointCloud, const cl_int* neighborIndices, int offset, int num_pts, float* L)
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
                L[L_INDEX(r,c)] += rowOfF.s[r] * rowOfF.s[c];
    }
}

void DescriptorMakerCpu::SetUpperRight(cl_float4 normal, float* L)
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

void DescriptorMakerCpu::SetLowerLeft(cl_float4 normal, float* L)
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

void DescriptorMakerCpu::SetRightVector(cl_float4 ctpoint, cl_float4 ctnormal
                                     , const cl_float4* pointCloud, const cl_int* neighborIndices, int offset, int num_pts, float* L)
{
    int nbidx;
    cl_float4 diff;
    cl_float4 fx[NUM_VAR];
    cl_float8 rowOfF;
    for(int i=0; i<NUM_VAR; i++)
        fx[i] = (cl_float4){0,0,0,0};

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
        L[L_INDEX(i,L_DIM)] = clDot(fx[i], ctnormal);
}

void DescriptorMakerCpu::SolveLinearEq(const int dim, float* Ab_io, float* x_out)
{
    int width = dim+1;

    for(int i=0; i<dim-1; i++)
    {
        // Search for maximum in this column
        float maxEl = fabsf(Ab_io[i*width+i]);
        int maxRow = i;
        for (int k=i+1; k<dim; k++)
        {
            if (fabsf(Ab_io[k*width+i]) > maxEl)
            {
                maxEl = fabsf(Ab_io[k*width+i]);
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        for (int k=i; k<width;k++)
        {
            float tmp = Ab_io[maxRow*width+k];
            Ab_io[maxRow*width+k] = Ab_io[i*width+k];
            Ab_io[i*width+k] = tmp;
        }

        // Make all rows below this one 0 in current column
        for (int k=i+1; k<dim; k++)
        {
            float c = -Ab_io[k*width+i]/Ab_io[i*width+i];
            for (int j=i; j<width; j++)
            {
                if (i==j)
                    Ab_io[k*width+j] = 0;
                else
                    Ab_io[k*width+j] += c * Ab_io[i*width+j];
            }
        }
    }

    // Solve equation Ax=b for an upper triangular matrix Ab_io
    for (int i=dim-1; i>=0; i--)
    {
        x_out[i] = Ab_io[i*width+dim]/Ab_io[i*width+i];
        for (int k=i-1;k>=0; k--)
            Ab_io[k*width+dim] -= Ab_io[k*width+i] * x_out[i];
    }
}

DescType DescriptorMakerCpu::GetDescriptorByEigenDecomp(float Avec[NUM_VAR])
{
    float egval[PT_DIM];
    float egvec[PT_DIM*PT_DIM];

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
    if(fabsf(egval[0]) <= fabsf(egval[2]) && fabsf(egval[1]) <= fabsf(egval[2]))
        first = 2;
    else if(fabsf(egval[0]) <= fabsf(egval[1]) && fabsf(egval[2]) <= fabsf(egval[1]))
        first = 1;
    SwapEigen(egval, egvec, first, 0);

    // swap second largest eigenvalue with eigenvalue[1]
    if(fabsf(egval[1]) < fabsf(egval[2]))
    {
        second = 2;
        SwapEigen(egval, egvec, second, 1);
    }

    // rescale descriptor and reverse sign of descriptor
    DescType descriptor;
    for(int i=0; i<PT_DIM; i++)
        descriptor.s[i] = -egval[i]*EQUATION_SCALE;
    return descriptor;
}

void DescriptorMakerCpu::SwapEigen(float egval[PT_DIM], float egvec[PT_DIM*PT_DIM], int src, int dst)
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

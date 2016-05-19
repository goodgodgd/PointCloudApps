#include "kernel_common.cl"

#ifndef	COMPUTE_DESCRIPTOR
#define COMPUTE_DESCRIPTOR
#include "eigen_decomp.cl"

#define NUM_VAR             6
#define PT_DIM              3
#define L_DIM               (NUM_VAR+PT_DIM)
#define L_WIDTH             (L_DIM+1)
#define L_INDEX(y,x)        ((y)*L_WIDTH+(x))
#define DESC_EQUATION_SIZE  L_DIM*L_WIDTH
#define EQUATION_SCALE      100.f


void set_eq_upper_left(float4 ctpoint, __read_only image2d_t pointimg
                , __global int* neighbor_indices, int offset, int num_pts, float* L)
{
    int width = get_image_width(pointimg);
    int nbidx;
    float4 diff, nbpoint;
    float rowOfF[NUM_VAR];

    for(int i=offset; i<offset+num_pts; i++)
    {
        nbidx = neighbor_indices[i];
        nbpoint = read_imagef(pointimg, image_sampler, (int2)(nbidx%width, nbidx/width));
        diff = nbpoint - ctpoint;
        diff = diff*EQUATION_SCALE;
        // set each row of F
        rowOfF[0] = diff.x*diff.x;
        rowOfF[1] = diff.y*diff.y;
        rowOfF[2] = diff.z*diff.z;
        rowOfF[3] = 2.f*diff.x*diff.y;
        rowOfF[4] = 2.f*diff.y*diff.z;
        rowOfF[5] = 2.f*diff.z*diff.x;

        // compute F'*F
        for(int r=0; r<NUM_VAR; r++)
            for(int c=0; c<NUM_VAR; c++)
                L[L_INDEX(r,c)] += rowOfF[r] * rowOfF[c];
    }
}

void set_eq_upper_right(float4 normal, float* L)
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

void set_eq_lower_left(float4 normal, float* L)
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

void set_eq_rhs(float4 ctpoint, float4 ctnormal, __read_only image2d_t pointimg
                    , __global int* neighbor_indices, int offset, int num_pts, float* L)
{
    int width = get_image_width(pointimg);
    int nbidx;
    float4 diff, nbpoint;
    float4 fx[NUM_VAR];
    float rowOfF[NUM_VAR];
    for(int i=0; i<NUM_VAR; i++)
        fx[i] = (float4){0,0,0,0};

    for(int i=offset; i<offset+num_pts; i++)
    {
        nbidx = neighbor_indices[i];
        nbpoint = read_imagef(pointimg, image_sampler, (int2)(nbidx%width, nbidx/width));
        diff = nbpoint - ctpoint;
        diff = diff*EQUATION_SCALE;
        // set each row of F
        rowOfF[0] = diff.x*diff.x;
        rowOfF[1] = diff.y*diff.y;
        rowOfF[2] = diff.z*diff.z;
        rowOfF[3] = 2.f*diff.x*diff.y;
        rowOfF[4] = 2.f*diff.y*diff.z;
        rowOfF[5] = 2.f*diff.z*diff.x;

        for(int r=0; r<NUM_VAR; r++)
        {
            fx[r].x += rowOfF[r] * diff.x;
            fx[r].y += rowOfF[r] * diff.y;
            fx[r].z += rowOfF[r] * diff.z;
        }
    }

    for(int i=0; i<NUM_VAR; i++)
        L[L_INDEX(i,L_DIM)] = dot(fx[i], ctnormal);
}


// linear equation Ax=b, Ab==[A b], out_x==x
void solve_linear_eq(float* Ab, float* out_x)
{
    int maxRow;
    float tmp, maxEl;

    for(int i=0; i<L_DIM; i++)
    {
        // Search for maximum in this column
        maxEl = fabs(Ab[L_INDEX(i,i)]);
        maxRow = i;

        for (int k=i+1; k<L_DIM; k++)
        {
            if (fabs(Ab[L_INDEX(k,i)]) > maxEl)
            {
                maxEl = fabs(Ab[L_INDEX(k,i)]);
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        for (int k=i; k<L_WIDTH;k++)
        {
            tmp = Ab[L_INDEX(maxRow,k)];
            Ab[L_INDEX(maxRow,k)] = Ab[L_INDEX(i,k)];
            Ab[L_INDEX(i,k)] = tmp;
        }

        // Make all rows below this one 0 in current column
        for (int k=i+1; k<L_DIM; k++)
        {
            tmp = -Ab[L_INDEX(k,i)]/Ab[L_INDEX(i,i)];
            for (int j=i; j<L_WIDTH; j++)
            {
                if (i==j)
                    Ab[L_INDEX(k,j)] = 0;
                else
                    Ab[L_INDEX(k,j)] += tmp * Ab[L_INDEX(i,j)];
            }
        }
    }
    // return;

    // Solve equation Ax=b for an upper triangular matrix Ab
    for (int i=L_DIM-1; i>=0; i--)
    {
        out_x[i] = Ab[L_INDEX(i,L_DIM)]/Ab[L_INDEX(i,i)];
        for (int k=i-1;k>=0; k--)
            Ab[L_INDEX(k,L_DIM)] -= Ab[L_INDEX(k,i)] * out_x[i];
    }
}

void swap_eigen(float egval[PT_DIM], float egvec[PT_DIM][PT_DIM], int src, int dst)
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
        tmp = egvec[i][dst];
        egvec[i][dst] = egvec[i][src];
        egvec[i][src] = tmp;
    }
}

float4 compute_descriptor_by_eigendecomp(float Avec[NUM_VAR])
{
    float egval[PT_DIM];
    float egvec[PT_DIM][PT_DIM];

    // convert solution to 3x3 matrix
    float Amat[PT_DIM][PT_DIM];
    Amat[0][0] = Avec[0];
    Amat[1][1] = Avec[1];
    Amat[2][2] = Avec[2];
    Amat[0][1] = Amat[1][0] = Avec[3];
    Amat[1][2] = Amat[2][1] = Avec[4];
    Amat[0][2] = Amat[2][0] = Avec[5];

    // eigen decomposition
    eigen_decomposition(Amat, egvec, egval);

    // sort eigenvalues and eigenvectors w.r.t eigenvalues
    int first=0, second=1;
    // swap first largest eigenvalue with eigenvalue[0]
    if(fabs(egval[0]) <= fabs(egval[2]) && fabs(egval[1]) <= fabs(egval[2]))
        first = 2;
    else if(fabs(egval[0]) <= fabs(egval[1]) && fabs(egval[2]) <= fabs(egval[1]))
        first = 1;
    swap_eigen(egval, egvec, first, 0);

    // swap second largest eigenvalue with eigenvalue[1]
    if(fabs(egval[1]) < fabs(egval[2]))
    {
        second = 2;
        swap_eigen(egval, egvec, second, 1);
    }

    // rescale descriptor and reverse sign of descriptor
    float4 descriptor = (float4)(-egval[0]*EQUATION_SCALE, -egval[1]*EQUATION_SCALE
                                , -egval[2]*EQUATION_SCALE, 0);
    return descriptor;
}

float compute_error(float* Ab, float* y)
{
    // set variables
    float FtF[NUM_VAR*NUM_VAR];
    float FtX[NUM_VAR];
    for(int r=0; r<NUM_VAR; r++)
    {
        FtX[r] = Ab[L_INDEX(r,L_DIM)];
        for(int c=0; c<NUM_VAR; c++)
            FtF[r*NUM_VAR+c] = Ab[L_INDEX(r,c)];
    }

    // compute first term
    float FtFy[NUM_VAR];
    for(int r=0; r<NUM_VAR; r++)
    {
        FtFy[r]=0;
        for(int c=0; c<NUM_VAR; c++)
            FtFy[r] += FtF[r*NUM_VAR+c]*y[r];
    }
    float first=0;
    for(int r=0; r<NUM_VAR; r++)
        first += FtFy[r]*y[r];

    // compute second term
    float second=0;
    for(int r=0; r<NUM_VAR; r++)
        second += FtX[r]*y[r];

    float error = 0.5f*first - second;
    return error;
}

__kernel void compute_descriptor(
                            __read_only image2d_t pointimg
                            , __read_only image2d_t normalimg
                            , __global int* neighbor_indices
                            , __global int* num_neighbors
                            , int max_numpts
							, __global float4* descriptors
                            , __global float* debug_buffer)
{
    unsigned int x = get_global_id(0);
    unsigned int y = get_global_id(1);
    int width = get_image_width(pointimg);
	int height = get_image_height(pointimg);
	int ptpos = y*width + x;
	int idcpos = ptpos*max_numpts;
    int numpts = num_neighbors[ptpos];
	float4 thispoint = read_imagef(pointimg, image_sampler, (int2)(x, y));
    float4 thisnormal = read_imagef(normalimg, image_sampler, (int2)(x, y));

    // check validity of point
    if(numpts < max_numpts/2)
    {
        descriptors[ptpos] = (float4)(0,0,0,0);
        return;
    }

    // matrix for linear equation, solution vector
    float linEq[DESC_EQUATION_SIZE];
    float ysol[L_DIM];
    for(int i=0; i<DESC_EQUATION_SIZE; i++)
        linEq[i] = 0;
    for(int i=0; i<L_DIM; i++)
        ysol[i] = 0;

    // set linear equation for matrix A
    // set F'*F part
    set_eq_upper_left(thispoint, pointimg, neighbor_indices, idcpos, numpts, linEq);
    // set G parts
    set_eq_upper_right(thisnormal, linEq);
    set_eq_lower_left(thisnormal, linEq);
    // set b in Ax=b
    set_eq_rhs(thispoint, thisnormal, pointimg, neighbor_indices, idcpos, numpts, linEq);

    solve_linear_eq(linEq, ysol);
    // descriptors[ptpos] = (float4)(ysol[0], ysol[1], ysol[2], ysol[3]);

    // compute shape descriptor by using eigen decomposition
    descriptors[ptpos] = compute_descriptor_by_eigendecomp(ysol);
    descriptors[ptpos].w = compute_error(linEq, ysol);
}

#endif // COMPUTE_DESCRIPTOR

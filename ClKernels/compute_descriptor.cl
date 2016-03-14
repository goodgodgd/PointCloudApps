#include "image_sampler.cl"

#ifndef	COMPUTE_DESCRIPTOR
#define COMPUTE_DESCRIPTOR

#define NUM_VAR     6
#define PT_DIM      3
#define L_DIM       (NUM_VAR+PT_DIM)
#define L_WIDTH     (L_DIM+1)
#define L_INDEX(y,x)    ((y)*L_WIDTH+(x))


void set_eq_upper_left(float4 centerpt
                    , __read_only image2d_t pointimg
                    , __global int* neigbor_indices
                    , int offset, int num_pts
                    , float* out_lineq)
{
    float4 neighbor, diff;
    float rowOfF[NUM_VAR];
    int width = get_image_width(pointimg);
    int nx, ny;

    for(int i=offset; i<offset+num_pts; i++)
    {
        ny = neigbor_indices[offset+i]/width;
        nx = neigbor_indices[offset+i]%width;
        neighbor = read_imagef(pointimg, image_sampler, (int2)(nx, ny));
        diff = neighbor - centerpt;

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
                out_lineq[L_INDEX(r,c)] += rowOfF[r] * rowOfF[c];
    }
}

void set_eq_upper_right(float4 normal, float* out_lineq)
{
    int bgx = 6;
    int bgy = 0;

    out_lineq[L_INDEX(bgy+0,bgx+0)] = normal.x;
    out_lineq[L_INDEX(bgy+3,bgx+0)] = normal.y;
    out_lineq[L_INDEX(bgy+5,bgx+0)] = normal.z;

    out_lineq[L_INDEX(bgy+3,bgx+1)] = normal.x;
    out_lineq[L_INDEX(bgy+1,bgx+1)] = normal.y;
    out_lineq[L_INDEX(bgy+4,bgx+1)] = normal.z;

    out_lineq[L_INDEX(bgy+5,bgx+2)] = normal.x;
    out_lineq[L_INDEX(bgy+4,bgx+2)] = normal.y;
    out_lineq[L_INDEX(bgy+2,bgx+2)] = normal.z;
}

void set_eq_lower_left(float4 normal, float* out_lineq)
{
    int bgx = 0;
    int bgy = 6;

    out_lineq[L_INDEX(bgy+0,bgx+0)] = normal.x;
    out_lineq[L_INDEX(bgy+0,bgx+3)] = normal.y;
    out_lineq[L_INDEX(bgy+0,bgx+5)] = normal.z;

    out_lineq[L_INDEX(bgy+1,bgx+3)] = normal.x;
    out_lineq[L_INDEX(bgy+1,bgx+1)] = normal.y;
    out_lineq[L_INDEX(bgy+1,bgx+4)] = normal.z;

    out_lineq[L_INDEX(bgy+2,bgx+5)] = normal.x;
    out_lineq[L_INDEX(bgy+2,bgx+4)] = normal.y;
    out_lineq[L_INDEX(bgy+2,bgx+2)] = normal.z;
}

void set_eq_rhs(float4 ctpoint, float4 ctnormal
                    , __read_only image2d_t pointimg
                    , __global int* neigbor_indices
                    , int offset, int num_pts
                    , float* out_lineq)
{
    float4 neighbor, diff;
    float rowOfF[NUM_VAR];
    int width = get_image_width(pointimg);
    int nx, ny;

    float4 fx[NUM_VAR];
    for(int i=0; i<NUM_VAR; i++)
        fx[i] = (float4){0,0,0,0};

    for(int i=offset; i<offset+num_pts; i++)
    {
        ny = neigbor_indices[offset+i]/width;
        nx = neigbor_indices[offset+i]%width;
        neighbor = read_imagef(pointimg, image_sampler, (int2)(nx, ny));
        diff = neighbor - ctpoint;
        // set each row of F
        rowOfF[0] = diff.x*diff.x;
        rowOfF[1] = diff.y*diff.y;
        rowOfF[2] = diff.z*diff.z;
        rowOfF[3] = 2.f*diff.x*diff.y;
        rowOfF[4] = 2.f*diff.y*diff.z;
        rowOfF[5] = 2.f*diff.z*diff.x;

        for(int i=0; i<NUM_VAR; i++)
        {
            fx[i].x += rowOfF[i] * diff.x;
            fx[i].y += rowOfF[i] * diff.y;
            fx[i].z += rowOfF[i] * diff.z;
        }
    }

    for(int i=0; i<NUM_VAR; i++)
        out_lineq[L_INDEX(i,L_DIM)] = dot(fx[i], ctnormal);
}


// linear equation Ax=b, Ab==[A b], out_x==x
void solve_linear_eq(float Ab[L_DIM][L_WIDTH], float out_x[L_DIM])
{
    int maxRow;
    float tmp, maxEl;

    for(int i=0; i<L_DIM; i++)
    {
        // Search for maximum in this column
        maxEl = fabs(Ab[i][i]);
        maxRow = i;

        for (int k=i+1; k<L_DIM; k++)
        {
            if (fabs(Ab[k][i]) > maxEl)
            {
                maxEl = fabs(Ab[k][i]);
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        for (int k=i; k<L_WIDTH;k++)
        {
            tmp = Ab[maxRow][k];
            Ab[maxRow][k] = Ab[i][k];
            Ab[i][k] = tmp;
        }

        // Make all rows below this one 0 in current column
        for (int k=i+1; k<L_DIM; k++)
        {
            tmp = -Ab[k][i]/Ab[i][i];
            for (int j=i; j<L_WIDTH; j++)
            {
                if (i==j)
                    Ab[k][j] = 0;
                else
                    Ab[k][j] += tmp * Ab[i][j];
            }
        }
    }
    // return;

    // Solve equation Ax=b for an upper triangular matrix Ab
    for (int i=L_DIM-1; i>=0; i--)
    {
        out_x[i] = Ab[i][L_DIM]/Ab[i][i];
        for (int k=i-1;k>=0; k--)
            Ab[k][L_DIM] -= Ab[k][i] * out_x[i];
    }
}



__kernel void compute_descriptor(
                            __read_only image2d_t pointimg		// width*height
                            , __read_only image2d_t normalimg	// width*height
                            , __global int* neigbor_indices		// width*height*max_numpts
                            , __global int* num_neighbors		// width*height
                            , int max_numpts
							, __global float4* descriptors)
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

    // matrix for linear equation, solution vector
    float linEq[L_DIM*L_WIDTH];
    float ysol[L_DIM];
    for(int i=0; i<L_DIM*L_WIDTH; i++)
        linEq[i] = i*i % 10;
    for(int i=0; i<L_DIM; i++)
        ysol[i] = 0;

    // set linear equation for matrix A
    // set F'*F part
    set_eq_upper_left(thispoint, pointimg, neigbor_indices, idcpos, numpts, linEq);
    // set G parts
    set_eq_upper_right(thisnormal, linEq);
    set_eq_lower_left(thisnormal, linEq);
    // set b in Ax=b
    set_eq_rhs(thispoint, thisnormal, pointimg, neigbor_indices, idcpos, numpts, linEq);

    solve_linear_eq(L_DIM, linEq, ysol);

}

#endif // COMPUTE_DESCRIPTOR

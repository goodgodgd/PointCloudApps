#include "image_sampler.cl"

#ifndef	NORMAL_VECTOR_PG
#define NORMAL_VECTOR_PG
#include "eigen_decomp.cl"

void make_covariance_from_offset(float4 centerpt, __global float4* points, int offset, int numpts, float covar[3][3])
{
	float4 ptdiff;
	// compute covariance
	for(int i=0; i<numpts; i++)
	{
		ptdiff = points[offset+i] - centerpt;
		covar[0][0] += ptdiff.x * ptdiff.x;
		covar[1][1] += ptdiff.y * ptdiff.y;
		covar[2][2] += ptdiff.z * ptdiff.z;
		covar[0][1] += ptdiff.x * ptdiff.y;
		covar[0][2] += ptdiff.x * ptdiff.z;
		covar[1][2] += ptdiff.y * ptdiff.z;
	}
	// set low components
	covar[1][0] = covar[0][1];
	covar[2][0] = covar[0][2];
	covar[2][1] = covar[1][2];
}

__kernel void compute_normal_vector_with_neighbor_points(
							__global float4* neighbor_points
                            , int max_numpts
							, __write_only image2d_t normalimg)
{
    unsigned int x = get_global_id(0);
    unsigned int y = get_global_id(1);
    int width = get_image_width(normalimg);
	int height = get_image_height(normalimg);
    int offset_pos = (y*width + x)*max_numpts;
    int numpts = (int)neighbor_points[offset_pos].w;
    float4 thepoint = neighbor_points[offset_pos];

    // skip if lack of neighbor points
	if(numpts < max_numpts/2)
	{
		write_imagef(normalimg, (int2)(x, y), (float4)(0,0,0,0));
		return;
	}

	// create covariance matrix
	float covar[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
	make_covariance_from_offset(thepoint, neighbor_points, offset_pos+1, numpts, covar);

	// eigendecomposition
	float evec[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
	float eval[3] = {0,0,0};
	eigen_decomposition(covar, evec, eval);

	// select normal vector
	float4 normalv;
	if(eval[0]<=eval[1] && eval[0]<=eval[2])
		normalv = (float4)(evec[0][0], evec[1][0], evec[2][0], 0);
	else if(eval[1]<=eval[0] && eval[1]<=eval[2])
		normalv = (float4)(evec[0][1], evec[1][1], evec[2][1], 0);
	else if(eval[2]<=eval[0] && eval[2]<=eval[1])
		normalv = (float4)(evec[0][2], evec[1][2], evec[2][2], 0);
	normalv = normalize(normalv);

    // align direction toward camera
    if(dot(normalv, thepoint) < 0)
        normalv = -normalv;

	write_imagef(normalimg, (int2)(x, y), normalv);
}

#endif // NORMAL_VECTOR_PG

#include "kernel_common.cl"

#ifndef	NORMAL_VECTOR
#define NORMAL_VECTOR
#include "eigen_decomp.cl"

void make_covariance_from_offset(__read_only image2d_t pointimg
								, __global int* neigbor_indices
								, int offset
								, int numpts
								, float radius
								, float out_covar[3][3])
{
	int width = get_image_width(pointimg);
	int height = get_image_height(pointimg);
	int nx, ny;
	float4 neighbor, ptdiff;
	float weight;
	float4 meanpt = (float4)(0,0,0,0);
	
	// compute mean
	for(int i=0; i<numpts; i++)
	{
		ny = neigbor_indices[offset+i]/width;
		nx = neigbor_indices[offset+i]%width;
		neighbor = read_imagef(pointimg, image_sampler, (int2)(nx, ny));
		meanpt += neighbor;
	}
	meanpt = meanpt/numpts;
	
	// compute covariance
	for(int i=0; i<numpts; i++)
	{
		ny = neigbor_indices[offset+i]/width;
		nx = neigbor_indices[offset+i]%width;
		neighbor = read_imagef(pointimg, image_sampler, (int2)(nx, ny));
		ptdiff = neighbor - meanpt;
		weight = (radius - length(ptdiff))*100.f;
		out_covar[0][0] += ptdiff.x * ptdiff.x * weight;
		out_covar[1][1] += ptdiff.y * ptdiff.y * weight;
		out_covar[2][2] += ptdiff.z * ptdiff.z * weight;
		out_covar[0][1] += ptdiff.x * ptdiff.y * weight;
		out_covar[0][2] += ptdiff.x * ptdiff.z * weight;
		out_covar[1][2] += ptdiff.y * ptdiff.z * weight;
	}
	
	// set low components
	out_covar[1][0] = out_covar[0][1];
	out_covar[2][0] = out_covar[0][2];
	out_covar[2][1] = out_covar[1][2];
}

__kernel void compute_normal_vector(
							__read_only image2d_t pointimg
							, __global int* neigbor_indices
							, __global int* num_neighbors
                            , int max_numpts
                            , float radius
							, __write_only image2d_t out_normalimg
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

    // skip if lack of neighbor points
	if(numpts < max_numpts/3)
	{
		write_imagef(out_normalimg, (int2)(x, y), (float4)(0,0,0,0));
		return;
	}

	// create covariance matrix
	float covar[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
	make_covariance_from_offset(pointimg, neigbor_indices, idcpos, numpts, radius, covar);

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
	
	int dbgcnt=0;
	if(x>=282 && x<=284 &&y==121)
	{
		debug_buffer[(x-282)*10 + dbgcnt++] = x;
		debug_buffer[(x-282)*10 + dbgcnt++] = eval[0];
		debug_buffer[(x-282)*10 + dbgcnt++] = eval[1];
		debug_buffer[(x-282)*10 + dbgcnt++] = eval[2];

		debug_buffer[(x-282)*10 + dbgcnt++] = evec[0][0];
		debug_buffer[(x-282)*10 + dbgcnt++] = evec[1][0];
		debug_buffer[(x-282)*10 + dbgcnt++] = evec[2][0];
		debug_buffer[(x-282)*10 + dbgcnt++] = evec[0][1];
		debug_buffer[(x-282)*10 + dbgcnt++] = evec[1][1];
		debug_buffer[(x-282)*10 + dbgcnt++] = evec[2][1];
	}

    // align direction toward camera
    if(dot(normalv, thispoint) > 0)
        normalv = -normalv;

	write_imagef(out_normalimg, (int2)(x, y), normalv);
}

#endif // NORMAL_VECTOR

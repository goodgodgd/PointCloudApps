#include "image_sampler.cl"

#ifndef	COMPUTE_DESCRIPTOR
#define COMPUTE_DESCRIPTOR

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

    descriptors[ptpos] = thispoint * 2.f;
}

#endif // COMPUTE_DESCRIPTOR

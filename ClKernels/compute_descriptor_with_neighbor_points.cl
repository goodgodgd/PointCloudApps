#include "image_sampler.cl"

#ifndef	COMPUTE_DESCRIPTOR_PG
#define COMPUTE_DESCRIPTOR_PG

__kernel void compute_descriptor_with_neighbor_points(
                            __global float4* neighbor_points
                            , int max_numpts
                            , __read_only image2d_t normalimg
							, __global float4* descriptors)
{
    unsigned int x = get_global_id(0);
    unsigned int y = get_global_id(1);

    float4 thepoint = read_imagef(normalimg, image_sampler, (int2)(x, y));
	thepoint = thepoint * 2.f;

}

#endif // COMPUTE_DESCRIPTOR_PG

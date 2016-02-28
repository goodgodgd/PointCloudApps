
#include "image_sampler.cl"

#ifndef	COMPUTE_DESCRIPTOR_PG
#define COMPUTE_DESCRIPTOR_PG

__kernel void compute_descriptor(__read_only image2d_t pointimg
                            , __read_only image2d_t normalimg
							, float radius
							, float focal_length
							, __global float4* descriptors)
{
    unsigned int x = get_global_id(0);
    unsigned int y = get_global_id(1);

//    float4 thepoint = read_imagef(pointimg, image_sampler, (int2)(x, y));

}

#endif // COMPUTE_DESCRIPTOR_PG

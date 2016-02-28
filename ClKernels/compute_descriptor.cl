
#ifndef	IMAGE_SAMPLER
#define IMAGE_SAMPLER
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE
                                     | CLK_ADDRESS_CLAMP_TO_EDGE
                                     | CLK_FILTER_NEAREST;
#endif // IMAGE_SAMPLER

#define MAX_NEIGHBOR_POINTS       30
#include "radius_search.cl"


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

#ifndef	IMAGE_SAMPLER
#define IMAGE_SAMPLER
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE
                                     | CLK_ADDRESS_CLAMP_TO_EDGE
                                     | CLK_FILTER_NEAREST;
#endif // IMAGE_SAMPLER

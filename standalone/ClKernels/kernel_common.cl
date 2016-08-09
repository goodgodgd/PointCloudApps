#ifndef	KERNEL_COMMON_CL
#define KERNEL_COMMON_CL
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE
                                     | CLK_ADDRESS_CLAMP_TO_EDGE
                                     | CLK_FILTER_NEAREST;

#define DEPTH(p)            (p.x)
#define DEAD_RANGE          0.1f
#define DEPTH_VALID(p)      (DEPTH(p) >= DEAD_RANGE)
#define DEPTH_INVALID(p)    (DEPTH(p) < DEAD_RANGE)

#endif // KERNEL_COMMON_CL

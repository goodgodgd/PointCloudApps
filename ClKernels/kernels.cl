
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE
                                     | CLK_ADDRESS_CLAMP_TO_EDGE
                                     | CLK_FILTER_NEAREST;

__kernel void mean_kernel(__read_only image2d_t iimage,
							__write_only image2d_t oimage)
{
    unsigned int x = get_global_id(0);
    unsigned int y = get_global_id(1);
    int halfWindow = 1;
    float4 pixelValue;
    float4 computedFilter=0.0f;
    int i, j;

        pixelValue = read_imagef(iimage, image_sampler, (int2)(x, y));
	write_imagef(oimage, (int2)(x, y), pixelValue);
	return;

    for(i=-halfWindow; i<=halfWindow; i++)
    {
       for(j=-halfWindow; j<=halfWindow; j++)
       {
           pixelValue = read_imagef(iimage, image_sampler, (int2)(x+i, y+j));
           computedFilter += pixelValue/9.f;
       }
    }
    write_imagef(oimage, (int2)(x, y), computedFilter);
}


#include "image_sampler.cl"

#ifndef SEARCH_NEIGHBOR
#define SEARCH_NEIGHBOR

inline int rand(uint seed, uint range)
{
    // ulong lrnd = (seed*0x5DEECE66DL + 0xBL) & ((1L<<48) - 1);
    // int irnd = (int)(lrnd>>16);
    uint value = seed;
    value = (value ^ 61) ^ (value>>16);
    value *= 9;
    value ^= value << 4;
    value *= 0x27d4eb2d;
    value ^= value >> 15;
    return (int)(value%range);
}

__kernel void search_neighbor_indices(__read_only image2d_t pointimg
                                    , float metric_radius
                					, float focal_length
                                    , int max_numpts
                					, __global int* out_indices
                                    , __global int* out_numnbs)
{
    unsigned int x = get_global_id(0);
    unsigned int y = get_global_id(1);
    int width = get_image_width(pointimg);
	int height = get_image_height(pointimg);
    float4 thispoint = read_imagef(pointimg, image_sampler, (int2)(x, y));
    int ptpos = y*width + x;
    int idcpos = ptpos*max_numpts;
    // initialize number of neighbor points
    out_numnbs[ptpos] = 0;

    if(thispoint.x < 0.01f)
        return;

	float4 sample_point;
	int numpts = 0;
    // convert radius in pixel
	int pixel_radius = round(metric_radius / thispoint.x * focal_length);
	pixel_radius = max(pixel_radius, 1);
    int rx, ry, seed=1, trycnt=0;
    uint rndrange = pixel_radius*2 + 1;

    while(numpts < max_numpts && trycnt < (int)(max_numpts*1.5f))
    {
        trycnt++;
        seed += 11;
        rx = x + rand(seed, rndrange) - pixel_radius;
        seed += 11;
        ry = y + rand(seed, rndrange) - pixel_radius;
        if(rx==x && ry==y)
            continue;

        // check if sample point is in-radius
        sample_point = read_imagef(pointimg, image_sampler, (int2)(rx, ry));
        if(distance(sample_point, thispoint) < metric_radius)
        {
            out_indices[idcpos + numpts] = ry*width + rx;
            numpts++;
        }
    }
    out_numnbs[ptpos] = numpts;
    return;







    float rf, cf;
	int ri, ci;

	// set loop interval
	float itv = (float)(pixel_radius*2+1) / sqrt((float)max_numpts) * 0.8f;
	itv = max(itv, 1.f);
    pixel_radius += 1;

    // set search range
    float row_min = max(round((float)(y-pixel_radius)), 0.f);
    float row_max = min(round((float)(y+pixel_radius)), (float)height-0.51f);
    float col_min = max(round((float)(x-pixel_radius)), 0.f);
    float col_max = min(round((float)(x+pixel_radius)), (float)width-0.51f);

	// search neighbor region
    for(rf=row_min; rf<row_max; rf+=itv)
    {
		if(numpts>=max_numpts)
			break;
		ri = round(rf);

		for(cf=col_min; cf<col_max; cf+=itv)
		{
			if(numpts>=max_numpts)
				break;
			ci = round(cf);

			if(ri==y && ci==x)
				continue;

			// check if sample point is in-radius
			sample_point = read_imagef(pointimg, image_sampler, (int2)(ci, ri));
            if(distance(sample_point, thispoint) < metric_radius)
			{
				out_indices[idcpos + numpts] = ri*width + ci;
				numpts++;
			}
		}
    }

    out_numnbs[ptpos] = numpts;
}

#endif // SEARCH_NEIGHBOR

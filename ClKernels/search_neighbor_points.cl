#include "image_sampler.cl"

#ifndef SEARCH_NEIGHBOR
#define SEARCH_NEIGHBOR

__kernel void search_neighbor_points(__read_only image2d_t pointimg
                                    , float metric_radius
                					, float focal_length
                                    , int max_numpts
                					, __global float4* out_points)
{
    unsigned int x = get_global_id(0);
    unsigned int y = get_global_id(1);
    int width = get_image_width(pointimg);
	int height = get_image_height(pointimg);
    float4 thepoint = read_imagef(pointimg, image_sampler, (int2)(x, y));
    int out_offset = (y*width + x)*max_numpts;

    // skip invalid point
    if(thepoint.x < 0.01f)
    {
        for(int i=0; i<max_numpts; i++)
            out_points[out_offset + i] = (float4)(0,0,0,0);
        return;
    }

	float4 sample_point;
	int numpts = 0;
	float rf, cf;
	int ri, ci;

    // set first point as thepoint
    out_points[out_offset] = thepoint;
    numpts = 1;

	// convert radius in pixel
	int pixel_radius = round(metric_radius / thepoint.x * focal_length);
	pixel_radius = max(pixel_radius, 1);
	// set loop interval
	float itv = (float)(pixel_radius*2+1) / sqrt((float)max_numpts) * 0.6f;
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
            if(distance(sample_point, thepoint) < metric_radius)
			{
				out_points[out_offset + numpts] = sample_point;
				numpts++;
			}
		}
    }

//    out_points[out_offset] = (float4)(pixel_radius, itv, numpts, metric_radius);
    out_points[out_offset].w = numpts-1;    // minus 1 for thepoint
}
#endif // SEARCH_NEIGHBOR

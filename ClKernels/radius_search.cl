#ifndef	IMAGE_SAMPLER
#define IMAGE_SAMPLER
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE
                                     | CLK_ADDRESS_CLAMP_TO_EDGE
                                     | CLK_FILTER_NEAREST;
#endif // IMAGE_SAMPLER

int radius_search(__read_only image2d_t pointimg, 
					int x, int y, 
					float focal_length, 
					float metric_radius, 
					float4* out_points, int maxnpts)
{
	int width = get_image_width(pointimg);
	int height = get_image_height(pointimg);
	float4 center_point = read_imagef(pointimg, image_sampler, (int2)(x, y));

	float4 sample_point;
	int npts = 0;
	float rf, cf;
	int ri, ci;

	// convert radius in pixel
	int pixel_radius = round(metric_radius / center_point.x * focal_length) + 2;
	pixel_radius = max(pixel_radius, 1);
	// set loop interval
	float itv = (float)(pixel_radius*2) / sqrt((float)maxnpts);
	itv = max(itv, 1.f);

//	out_points[0] = (float4)(metric_radius, center_point.x, pixel_radius, itv);
//	return 1;


	// search neighbor region
    for(rf=(float)(y-pixel_radius); rf<=(float)(y+pixel_radius); rf+=itv)
    {
		if(npts>=maxnpts)
			break;

		ri = round(rf+0.5f);
		if(ri<0 || ri>=height)
			continue;

		for(cf=(float)(x-pixel_radius); cf<=(float)(x+pixel_radius); cf+=itv)
		{
			if(npts>=maxnpts)
				break;

			ci = round(cf+0.5f);
			if(ci<0 || ci>=width)
				continue;

			if(ri==y && ci==x)
				continue;

			// check if sample point is in-radius
			sample_point = read_imagef(pointimg, image_sampler, (int2)(ci, ri));
			if(distance(sample_point, center_point) < metric_radius)
			{
				out_points[npts] = sample_point;
				npts++;
			}
		}

    }
    return npts;
}


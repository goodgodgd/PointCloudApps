#include "image_sampler.cl"

__kernel void search_neighbor_indices(__read_only image2d_t pointimg
                                    , float metric_radius
                					, float focal_length
                                    , int max_numpts
                					, __global int* neibindices_out
                                    , __global int* numneibs_out
                                    , __global float* debug_buffer)
{
    int xid = get_global_id(0);
    int yid = get_global_id(1);
    int width = get_image_width(pointimg);
	int height = get_image_height(pointimg);
    float4 thispoint = read_imagef(pointimg, image_sampler, (int2)(xid, yid));
    int ptpos = yid*width + xid;
    int idcpos = ptpos*max_numpts;
    // initialize number of neighbor points
    numneibs_out[ptpos] = 0;

    if(thispoint.x < 0.01f)
        return;

	float4 sample_point;
	int numpts = 0;
    float rf, cf;
	int ri, ci;
    // convert radius in pixel
	int pixel_radius = round(metric_radius / thispoint.x * focal_length);
	pixel_radius = max(pixel_radius, (int)1);

	// set loop interval
	float itv = (float)(pixel_radius*2+1) / sqrt((float)max_numpts) * 0.8f;
	itv = max(itv, 1.f);

    // set search range
    float row_min = (float)max(yid-pixel_radius, (int)0);
    float row_max = (float)min(yid+pixel_radius, (int)(height-1));
    float col_min = (float)max(xid-pixel_radius, (int)0);
    float col_max = (float)min(xid+pixel_radius, (int)(width-1));
/*
    int dbg_count = 0;
    if(xid==191 && yid==136)
    {
        for(int i=0; i<100; i++)
            debug_buffer[i] = -1;
        debug_buffer[dbg_count++] = pixel_radius;
        debug_buffer[dbg_count++] = itv;
        debug_buffer[dbg_count++] = col_min;
        debug_buffer[dbg_count++] = col_max;
        debug_buffer[dbg_count++] = row_min;
        debug_buffer[dbg_count++] = row_max;
    }
*/
	// search neighbor region
    for(rf=row_min; rf<=row_max; rf+=itv)
    {
		if(numpts>=max_numpts)
			break;
		ri = round(rf);

		for(cf=col_min; cf<=col_max; cf+=itv)
		{
			if(numpts>=max_numpts)
				break;
			ci = round(cf);
			if(ri==yid && ci==xid)
				continue;

			// check if sample point is in-radius
			sample_point = read_imagef(pointimg, image_sampler, (int2)(ci, ri));
            if(distance(sample_point, thispoint) < metric_radius)
			{
				neibindices_out[idcpos + numpts] = ri*width + ci;
				numpts++;
			}
/*
            if(xid==191 && yid==136 && dbg_count < 90 && (xid-ci)<3 && (yid-ri)>2)
            {
                debug_buffer[dbg_count++] = ci;
                debug_buffer[dbg_count++] = ri;
                debug_buffer[dbg_count++] = distance(sample_point, thispoint);
            }
*/
		}
    }

    numneibs_out[ptpos] = numpts;
    return;
}

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
		}
    }

    numneibs_out[ptpos] = numpts;
    return;

    if(numpts > max_numpts/2 || itv < 1.5f)
        return;

    // search more neighbors around searched points
    int ni, numadd=0;
    for(int i=0; i<numpts; i++)
    {
        if(numpts + numadd>=max_numpts)
			break;

        // index of low right point
        ni = neibindices_out[idcpos + i] + width + 1;
        ri = ni/width;
        ci = ni%width;
        if(ri==yid && ci==xid)
            continue;

        // check if sample point is in-radius
        sample_point = read_imagef(pointimg, image_sampler, (int2)(ci, ri));
        if(distance(sample_point, thispoint) < metric_radius)
        {
            neibindices_out[idcpos + numpts + numadd] = ni;
            numadd++;
        }
    }

    numneibs_out[ptpos] = numpts + numadd;
}

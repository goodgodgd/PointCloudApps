#include "kernel_common.cl"

inline float get_mean_depth(__read_only image2d_t pointimg, const int xid, const int yid
                        , const float center_depth, const float diff_thresh)
{
    float4 point3d = read_imagef(pointimg, image_sampler, (int2)(xid, yid));
    // return point directly when point is valid
    if(DEPTH_VALID(point3d) && fabs(DEPTH(point3d) - center_depth) < diff_thresh)
        return DEPTH(point3d);

    // for invalid point, compute mean point of adjacent pixels
    int count=0;
    float mean_depth=0.f;

    point3d = read_imagef(pointimg, image_sampler, (int2)(xid+1, yid));
    if(DEPTH_VALID(point3d) && fabs(DEPTH(point3d) - center_depth) < diff_thresh)
    {
        mean_depth += DEPTH(point3d);
        count++;
    }
    point3d = read_imagef(pointimg, image_sampler, (int2)(xid-1, yid));
    if(DEPTH_VALID(point3d) && fabs(DEPTH(point3d) - center_depth) < diff_thresh)
    {
        mean_depth += DEPTH(point3d);
        count++;
    }
    point3d = read_imagef(pointimg, image_sampler, (int2)(xid, yid+1));
    if(DEPTH_VALID(point3d) && fabs(DEPTH(point3d) - center_depth) < diff_thresh)
    {
        mean_depth += DEPTH(point3d);
        count++;
    }
    point3d = read_imagef(pointimg, image_sampler, (int2)(xid, yid-1));
    if(DEPTH_VALID(point3d) && fabs(DEPTH(point3d) - center_depth) < diff_thresh)
    {
        mean_depth += DEPTH(point3d);
        count++;
    }

    if(count==0)
        mean_depth = 0.f;
    else
        mean_depth /= (float)count;
    return mean_depth;
}

bool get_around_depths(__read_only image2d_t pointimg
                 , const int xid, const int yid, int pxradius
                 , const float center_depth, const float focal_length
                 , float depths_out[4])
{
    const int width = get_image_width(pointimg);
	const int height = get_image_height(pointimg);
    if(xid < pxradius+1 || xid > width-pxradius-2 || yid < pxradius+1 || yid > height-pxradius-2)
        return false;

    const float depth_diff_thresh = (float)(pxradius)*center_depth/focal_length * 4.f;

    depths_out[0] = get_mean_depth(pointimg, xid, yid-pxradius, center_depth, depth_diff_thresh);
    if(depths_out[0] < DEAD_RANGE)
        return false;
    depths_out[1] = get_mean_depth(pointimg, xid, yid+pxradius, center_depth, depth_diff_thresh);
    if(depths_out[1] < DEAD_RANGE)
        return false;
    depths_out[2] = get_mean_depth(pointimg, xid-pxradius, yid, center_depth, depth_diff_thresh);
    if(depths_out[2] < DEAD_RANGE)
        return false;
    depths_out[3] = get_mean_depth(pointimg, xid+pxradius, yid, center_depth, depth_diff_thresh);
    if(depths_out[3] < DEAD_RANGE)
        return false;

    return true;
}

// #define DEBUG_SEARCH
#define DEBUG_LEN   100
#define DEBUG_X     252
#define DEBUG_Y     102

__kernel void search_neighbor_indices(__read_only image2d_t pointimg
                                    , float metric_radius
                					, float focal_length
                                    , int max_numpts
                					, __global int* neibindices_out
                                    , __global int* numneibs_out
                                    , __global float* debug_buffer)
{
    const int xid = get_global_id(0);
    const int yid = get_global_id(1);
    const int width = get_image_width(pointimg);
	const int height = get_image_height(pointimg);
    float4 thispoint = read_imagef(pointimg, image_sampler, (int2)(xid, yid));
    int ptpos = yid*width + xid;
    int idcpos = ptpos*max_numpts;
    // initialize number of neighbor points
    numneibs_out[ptpos] = 0;
#ifndef DEBUG_SEARCH
    if(xid==DEBUG_X && yid==DEBUG_Y)
    {
        for(int i=0; i<DEBUG_LEN; i++)
            debug_buffer[i] = -1.f;
    }
#endif

    // check depth validity
    if(DEPTH_INVALID(thispoint))
        return;

    float meter_to_pixel = focal_length / DEPTH(thispoint);
	int check_radius_px = round(metric_radius / 2.f * meter_to_pixel);
	check_radius_px = max(check_radius_px, 2);

    int dbg_count = 0;

    // check if thispoint is at border and get 4 points: up, down, left, right
    float around_depths[4];
    for(int i=0; i<4; i++)
        around_depths[i] = 0.f;
    if(get_around_depths(pointimg, xid, yid, check_radius_px, DEPTH(thispoint), focal_length, around_depths)==false)
    {
#ifdef DEBUG_SEARCH
        if(xid==DEBUG_X && yid==DEBUG_Y)
        {
            float4 dbg_point = read_imagef(pointimg, image_sampler, (int2)(xid, yid-check_radius_px));
            for(int i=0; i<DEBUG_LEN; i++)
                debug_buffer[i] = -1.f;
            debug_buffer[dbg_count++] = 1;
            debug_buffer[dbg_count++] = 1;
            debug_buffer[dbg_count++] = (float)(check_radius_px);
            debug_buffer[dbg_count++] = (float)(check_radius_px)*DEPTH(thispoint)/focal_length * 4.f;
            debug_buffer[dbg_count++] = DEPTH(dbg_point);
            debug_buffer[dbg_count++] = DEPTH(thispoint);
            debug_buffer[dbg_count++] = around_depths[0];
            debug_buffer[dbg_count++] = around_depths[1];
            debug_buffer[dbg_count++] = around_depths[2];
            debug_buffer[dbg_count++] = around_depths[3];
        }
#endif
        return;
    }

    float flat_diam_m = 2.f * check_radius_px / meter_to_pixel;
    // compute vertical search range and interval
    float ver_depth_diff = around_depths[0] - around_depths[1];
    float ver_flat_diam_m = 4.f*metric_radius*metric_radius
                            / (1.f + ver_depth_diff*ver_depth_diff/flat_diam_m/flat_diam_m);
    ver_flat_diam_m = sqrt(ver_flat_diam_m);
    float ver_radius_px = ver_flat_diam_m / 2.f * meter_to_pixel;
    float row_min = max((float)yid - ver_radius_px, 0.f);
    float row_max = min((float)yid + ver_radius_px, (float)(height-1));
    float vitv = ver_radius_px*2.f / sqrt((float)max_numpts);
    vitv = max(vitv, 1.f);

    // compute vertical search range and interval
    float hor_depth_diff = around_depths[2] - around_depths[3];
    float hor_flat_diam_m = 4.f*metric_radius*metric_radius
                            / (1.f + hor_depth_diff*hor_depth_diff/flat_diam_m/flat_diam_m);
    hor_flat_diam_m = sqrt(hor_flat_diam_m);
    float hor_radius_px = hor_flat_diam_m / 2.f * meter_to_pixel;
    float col_min = max((float)xid - hor_radius_px, 0.f);
    float col_max = min((float)xid + hor_radius_px, (float)(width-1));
    float hitv = hor_radius_px*2.f / sqrt((float)max_numpts);
    hitv = max(hitv, 1.f);

#ifdef DEBUG_SEARCH
    if(xid==DEBUG_X && yid==DEBUG_Y)
    {
        for(int i=0; i<DEBUG_LEN; i++)
            debug_buffer[i] = -1.f;
        debug_buffer[dbg_count++] = 2;
        debug_buffer[dbg_count++] = DEPTH(thispoint);
        debug_buffer[dbg_count++] = around_depths[0];
        debug_buffer[dbg_count++] = around_depths[1];
        debug_buffer[dbg_count++] = around_depths[2];
        debug_buffer[dbg_count++] = around_depths[3];
        debug_buffer[dbg_count++] = flat_diam_m;
        debug_buffer[dbg_count++] = 0;
        debug_buffer[dbg_count++] = ver_depth_diff;
        debug_buffer[dbg_count++] = ver_flat_diam_m;
        debug_buffer[dbg_count++] = ver_radius_px;
        debug_buffer[dbg_count++] = row_min;
        debug_buffer[dbg_count++] = row_max;
        debug_buffer[dbg_count++] = vitv;
        debug_buffer[dbg_count++] = 0;
        debug_buffer[dbg_count++] = hor_depth_diff;
        debug_buffer[dbg_count++] = hor_flat_diam_m;
        debug_buffer[dbg_count++] = hor_radius_px;
        debug_buffer[dbg_count++] = col_min;
        debug_buffer[dbg_count++] = col_max;
        debug_buffer[dbg_count++] = hitv;
        debug_buffer[dbg_count++] = 0;
        sdfsfd;
        sdfsdfsdf;
    }
#endif

    float4 sample_point;
    int numpts = 0;
    float rf, cf;
    int ri, ci;

	// search neighbor region
    for(rf=row_min; rf<row_max; rf+=vitv)
    {
		if(numpts>=max_numpts)
			break;
		ri = round(rf);

		for(cf=col_min; cf<col_max; cf+=hitv)
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
#ifdef DEBUG_SEARCH
            if(xid==DEBUG_X && yid==DEBUG_Y && dbg_count < 90)
            {
                debug_buffer[dbg_count++] = ci;
                debug_buffer[dbg_count++] = ri;
                debug_buffer[dbg_count++] = distance(sample_point, thispoint);
                debug_buffer[dbg_count++] = 0;
            }
#endif
		}
    }

    numneibs_out[ptpos] = numpts;
    return;
}

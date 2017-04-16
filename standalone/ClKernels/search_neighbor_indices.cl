#include "kernel_common.cl"


int2 find_edge_pixel(__read_only image2d_t pointimg, const int2 begpx, const int2 endpx, const float metric_radius)
{
    const float4 thispoint = read_imagef(pointimg, image_sampler, endpx);
    const int width = get_image_width(pointimg);
	const int height = get_image_height(pointimg);
    int xbeg = clamp(begpx.x, 0, width-1);
    int xend = clamp(endpx.x, 0, width-1);
    int ybeg = clamp(begpx.y, 0, height-1);
    int yend = clamp(endpx.y, 0, height-1);
    float4 sample_point;

	if(xbeg==xend)
	{
		if(ybeg<yend)
		{
			for(int yi=ybeg; yi<=yend; yi++)
		    {
		        sample_point = read_imagef(pointimg, image_sampler, (int2)(xbeg, yi));
		        if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
		            return (int2)(xbeg, yi);
		    }
		}
		else if(ybeg>yend)
		{
			for(int yi=ybeg; yi>=yend; yi--)
		    {
		        sample_point = read_imagef(pointimg, image_sampler, (int2)(xbeg, yi));
		        if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
		            return (int2)(xbeg, yi);
		    }
		}
	}

	if(ybeg==yend)
	{
		if(xbeg<xend)
		{
			for(int xi=xbeg; xi<=xend; xi++)
		    {
		        sample_point = read_imagef(pointimg, image_sampler, (int2)(xi, ybeg));
		        if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
		            return (int2)(xi, ybeg);
		    }
		}
		else if(xbeg>xend)
		{
			for(int xi=xbeg; xi>=xend; xi--)
		    {
		        sample_point = read_imagef(pointimg, image_sampler, (int2)(xi, ybeg));
		        if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
		            return (int2)(xi, ybeg);
		    }
		}
	}

    return endpx;
}

#define DEBUG_SEARCH
#define DEBUG_LEN   100
#define DEBUG_X     140
#define DEBUG_Y     140

__kernel void search_neighbor_indices(__read_only image2d_t pointimg
                                    , float metric_radius
                					, float focal_length
                                    , int neigb_limit
                					, __global int* neibindices_out
                                    , __global int* numneibs_out
                                    , int max_numpts
                                    , __global float* debug_buffer)
{
    const int cxid = get_global_id(0);
    const int cyid = get_global_id(1);
    const int width = get_image_width(pointimg);
	const int height = get_image_height(pointimg);
    const int2 thispixel = (int2)(cxid, cyid);
    const float4 thispoint = read_imagef(pointimg, image_sampler, thispixel);
    float thisdepth = DEPTH(thispoint);
    int ptpos = cyid*width + cxid;
    int indicespos = ptpos*max_numpts;
    const int pixel_radius = round(metric_radius/thisdepth*focal_length)+1;
    numneibs_out[ptpos] = 0;
    float4 sample_point;


    if(cxid==DEBUG_X && cyid==DEBUG_Y)
    {
        debug_buffer[0] = 0.f;
        int dbgcnt = (int)debug_buffer[0];
        debug_buffer[dbgcnt++] = 0;
        debug_buffer[dbgcnt++] = (float)pixel_radius;
        debug_buffer[dbgcnt++] = (float)cxid;
        debug_buffer[dbgcnt++] = (float)cyid;
        debug_buffer[dbgcnt++] = (float)thispoint.x;
        debug_buffer[dbgcnt++] = (float)thispoint.y;
        debug_buffer[dbgcnt++] = (float)thispoint.z;
        debug_buffer[0] = (float)dbgcnt;
    }

    if(DEPTH_INVALID(thispoint))
        return;

    int2 xhi_edge = find_edge_pixel(pointimg, (int2)(cxid+pixel_radius,cyid), thispixel, metric_radius);
    int2 xlo_edge = find_edge_pixel(pointimg, (int2)(cxid-pixel_radius,cyid), thispixel, metric_radius);
    int2 yhi_edge = find_edge_pixel(pointimg, (int2)(cxid,cyid+pixel_radius), thispixel, metric_radius);
    int2 ylo_edge = find_edge_pixel(pointimg, (int2)(cxid,cyid-pixel_radius), thispixel, metric_radius);

    if(cxid==DEBUG_X && cyid==DEBUG_Y)
    {
        int dbgcnt = (int)debug_buffer[0];
        debug_buffer[dbgcnt++] = 0;
        debug_buffer[dbgcnt++] = (float)xhi_edge.x;
        debug_buffer[dbgcnt++] = (float)xlo_edge.x;
        debug_buffer[dbgcnt++] = (float)xhi_edge.y;
        debug_buffer[dbgcnt++] = 0;
        debug_buffer[dbgcnt++] = (float)yhi_edge.y;
        debug_buffer[dbgcnt++] = (float)ylo_edge.y;
        debug_buffer[dbgcnt++] = (float)yhi_edge.x;
        debug_buffer[0] = (float)dbgcnt;
    }
    
    sample_point = read_imagef(pointimg, image_sampler, xhi_edge);
	if(distance(sample_point, thispoint) < metric_radius*0.5f)
		return;
    sample_point = read_imagef(pointimg, image_sampler, xlo_edge);
	if(distance(sample_point, thispoint) < metric_radius*0.5f)
		return;
    sample_point = read_imagef(pointimg, image_sampler, yhi_edge);
	if(distance(sample_point, thispoint) < metric_radius*0.5f)
		return;
    sample_point = read_imagef(pointimg, image_sampler, ylo_edge);
	if(distance(sample_point, thispoint) < metric_radius*0.5f)
		return;

    float divider = sqrt((float)neigb_limit)/2.f;
    float xhi_itv = (float)(xhi_edge.x - cxid) / divider;
    float xlo_itv = (float)(xlo_edge.x - cxid) / divider;
    float yhi_itv = (float)(yhi_edge.y - cyid) / divider;
    float ylo_itv = (float)(ylo_edge.y - cyid) / divider;


    // if(cxid==DEBUG_X && cyid==DEBUG_Y)
    // {
    //     int dbgcnt = (int)debug_buffer[0];
    //     debug_buffer[dbgcnt++] = 0;
    //     debug_buffer[dbgcnt++] = divider;
    //     debug_buffer[dbgcnt++] = xhi_itv;
    //     debug_buffer[dbgcnt++] = xlo_itv;
    //     debug_buffer[dbgcnt++] = yhi_itv;
    //     debug_buffer[dbgcnt++] = ylo_itv;
    //     debug_buffer[0] = (float)dbgcnt;
    // }

    float xf, yf;
    int xi, yi;
    int numpts = 0;

    neibindices_out[indicespos + numpts] = cyid*width + cxid;
    numpts++;

    // right line
    for(xf=(float)cxid+xhi_itv; xf<(float)xhi_edge.x; xf+=xhi_itv)
    {
        xi = round(xf);
        sample_point = read_imagef(pointimg, image_sampler, (int2)(xi, cyid));
        if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
        {
            neibindices_out[indicespos + numpts] = cyid*width + xi;
            numpts++;
        }
    }

    // left line
    for(xf=(float)cxid+xlo_itv; xf>(float)xlo_edge.x; xf+=xlo_itv)
    {
        xi = round(xf);
        sample_point = read_imagef(pointimg, image_sampler, (int2)(xi, cyid));
        if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
        {
            neibindices_out[indicespos + numpts] = cyid*width + xi;
            numpts++;
        }
    }

    // high line
    for(yf=(float)cyid+yhi_itv; yf<(float)yhi_edge.y; yf+=yhi_itv)
    {
        yi = round(yf);
        sample_point = read_imagef(pointimg, image_sampler, (int2)(cxid, yi));
        if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
        {
            neibindices_out[indicespos + numpts] = yi*width + cxid;
            numpts++;
        }
    }

    // low line
    for(yf=(float)cyid+ylo_itv; yf>(float)ylo_edge.y; yf+=ylo_itv)
    {
        yi = round(yf);
        sample_point = read_imagef(pointimg, image_sampler, (int2)(cxid, yi));
        if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
        {
            neibindices_out[indicespos + numpts] = yi*width + cxid;
            numpts++;
        }
    }

    // high region
    for(yf=(float)cyid+yhi_itv; yf<(float)yhi_edge.y; yf+=yhi_itv)
    {
        yi = round(yf);
        // high right region
        for(xf=(float)cxid+xhi_itv; xf<(float)xhi_edge.x; xf+=xhi_itv)
        {
            if(numpts >= neigb_limit)
                break;
            xi = round(xf);
            sample_point = read_imagef(pointimg, image_sampler, (int2)(xi, yi));
            if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
            {
                neibindices_out[indicespos + numpts] = yi*width + xi;
                numpts++;
            }
        }
        // high left region
        for(xf=(float)cxid+xlo_itv; xf>(float)xlo_edge.x; xf+=xlo_itv)
        {
            if(numpts >= neigb_limit)
                break;
            xi = round(xf);
            sample_point = read_imagef(pointimg, image_sampler, (int2)(xi, yi));
            if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
            {
                neibindices_out[indicespos + numpts] = yi*width + xi;
                numpts++;
            }
        }
    }

    // low region
    for(yf=(float)cyid+ylo_itv; yf>(float)ylo_edge.y; yf+=ylo_itv)
    {
        yi = round(yf);
        // low right region
        for(xf=(float)cxid+xhi_itv; xf<(float)xhi_edge.x; xf+=xhi_itv)
        {
            if(numpts >= neigb_limit)
                break;
            xi = round(xf);
            sample_point = read_imagef(pointimg, image_sampler, (int2)(xi, yi));
            if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
            {
                neibindices_out[indicespos + numpts] = yi*width + xi;
                numpts++;
            }
        }
        // low left region
        for(xf=(float)cxid+xlo_itv; xf>(float)xlo_edge.x; xf+=xlo_itv)
        {
            if(numpts >= neigb_limit)
                break;
            xi = round(xf);
            sample_point = read_imagef(pointimg, image_sampler, (int2)(xi, yi));
            if(DEPTH_VALID(sample_point) && distance(sample_point, thispoint) < metric_radius)
            {
                neibindices_out[indicespos + numpts] = yi*width + xi;
                numpts++;
            }
        }
    }

    numneibs_out[ptpos] = numpts;
    return;
}

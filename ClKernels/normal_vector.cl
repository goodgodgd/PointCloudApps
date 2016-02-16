#ifndef	IMAGE_SAMPLER
#define IMAGE_SAMPLER
__constant sampler_t image_sampler = CLK_NORMALIZED_COORDS_FALSE
                                     | CLK_ADDRESS_CLAMP_TO_EDGE
                                     | CLK_FILTER_NEAREST;
#endif // IMAGE_SAMPLER

#define MAX_POINTS	50

__kernel void normal_vector(__read_only image2d_t pointimg
							, float radius
							, float focal_length
							, __write_only image2d_t normalimg)
{
    unsigned int x = get_global_id(0);
    unsigned int y = get_global_id(1);

	float4 thepoint = read_imagef(pointimg, image_sampler, (int2)(x, y));
	
	// search in-radius points
	float4 inr_points[MAX_POINTS];
	int npts;//  = radius_search(pointimg, x, y, focal_length, radius, inr_points, MAX_POINTS);

	float4 output = (float4)(0,0,0, (float)npts);
	write_imagef(normalimg, (int2)(x, y), inr_points[0]);
	return;

/*
	// create covariance matrix
	float covar[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
	make_covariance(thepoint, inr_points, npts, covar);

	// eigendecomposition
	float evec[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
	float eval[3] = {0,0,0};
	eigen_decomposition(covar, evec, eval);

	// select normal vector
	float4 normalv;
	if(eval[0]<=eval[1] && eval[0]<=eval[2])
		normalv = (float4)(evec[0][0], evec[1][0], evec[2][0], eval[0]);
	else if(eval[1]<=eval[0] && eval[1]<=eval[2])
		normalv = (float4)(evec[0][1], evec[1][1], evec[2][1], eval[1]);
	else if(eval[2]<=eval[0] && eval[2]<=eval[1])
		normalv = (float4)(evec[0][2], evec[1][2], evec[2][2], eval[2]);
*/
}


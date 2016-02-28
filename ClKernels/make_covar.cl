#ifndef	MAKE_COVARIANCE
#define MAKE_COVARIANCE

void make_covariance(float4 ctpoint, float4* nbpoints, int npts, float covar[3][3])
{
	float4 ptdiff;
	// compute mean
	for(int i=0; i<npts; i++)
	{
		ptdiff = nbpoints[i] - ctpoint;
		covar[0][0] += ptdiff.x * ptdiff.x;
		covar[1][1] += ptdiff.y * ptdiff.y;
		covar[2][2] += ptdiff.z * ptdiff.z;
		covar[0][1] += ptdiff.x * ptdiff.y;
		covar[0][2] += ptdiff.x * ptdiff.z;
		covar[1][2] += ptdiff.y * ptdiff.z;
	}
	// set low components
	covar[1][0] = covar[0][1];
	covar[2][0] = covar[0][2];
	covar[2][1] = covar[1][2];
}

#endif // MAKE_COVARIANCE

#include "SiftFeature.h"


double sift_feature_distance( const SiftFeature &a, const SiftFeature &b )
{
	double result = 0;
	for(int i=0; i<128; i++)
		result += (a.descriptor[i] - b.descriptor[i]) * (a.descriptor[i] - b.descriptor[i]);
	return result;
}


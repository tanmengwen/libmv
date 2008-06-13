#ifndef __SIFTFEATURE_H__
#define __SIFTFEATURE_H__


#include "PointFeature.h"

class SiftFeature : public PointFeature
{
public:
	int descriptor[128];
};

double sift_feature_distance( const SiftFeature &a, const SiftFeature &b );


#endif //__SIFTFEATURE_H__


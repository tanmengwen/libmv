#ifndef __POINTTRACK_H__
#define __POINTTRACK_H__

#include "Frame.h"
#include <map>


class PointTrack
{
public:
	std::map<Frame*,int> track_indices;  // track_indices[i] == feature index of the track at image i

//	bool operator== (PointTrack i) const;
};


#endif //__POINTTRACK_H__


#ifndef __TRACKEDSEQUENCE_H__
#define __TRACKEDSEQUENCE_H__


#include "Frame.h"
#include "TwoViewMatches.h"
#include "PointTrack.h"

#include <vector>
#include <list>

class TrackedSequence
{
public:
	std::vector<Frame> frames;
	std::vector<TwoViewMatches> two_view_matches;
	std::list<PointTrack> tracks;

	void load_frames(std::vector<std::string> &filenames);
	void compute_sift_features();
	
	void compute_two_view_matches();
	void compute_track_from_matches();
};


#endif //__TRACKEDSEQUENCE_H__


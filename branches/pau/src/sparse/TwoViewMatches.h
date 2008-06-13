#ifndef __TWOVIEWMATCHES_H__
#define __TWOVIEWMATCHES_H__


#include "Frame.h"
#include "PointMatch.h"

class TwoViewMatches
{
public:	
	Frame *frame[2];
	std::vector<PointMatch> candidate_matches;
	std::vector<PointMatch> robust_matches;

public:
	TwoViewMatches() {
	}
	
	TwoViewMatches(Frame &f0, Frame &f1) {
		set_frames(f0,f1);
	}
	void set_frames(Frame &f0, Frame &f1) {
		frame[0] = &f0;
		frame[1] = &f1;
	}
	
	void compute();
	bool is_good_enough();
private:
	void compute_candidate_matches();
	void compute_fundametal_matrixRANSAC();
};


#endif //__TWOVIEWMATCHES_H__


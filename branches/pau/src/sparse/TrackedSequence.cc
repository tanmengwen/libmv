#include "TrackedSequence.h"
#include "Frame.h"

	
void TrackedSequence::load_frames(std::vector<std::string> &filenames)
{
	for(unsigned i=0; i<filenames.size(); i++) {
		Frame f(filenames[i]);
		frames.push_back(f);
	}
}

void TrackedSequence::compute_sift_features()
{
	for(unsigned i=0; i<frames.size(); i++) {
		frames[i].load_sift_features();
	}
}
	
void TrackedSequence::compute_two_view_matches()
{
	for(unsigned i=0; i<frames.size(); i++)
	for(unsigned j=i+1; j<frames.size(); j++)
	{
		TwoViewMatches tvm(frames[i],frames[j]);
		tvm.compute();
		if(tvm.is_good_enough())
			two_view_matches.push_back(tvm);
	}
}

void TrackedSequence::compute_track_from_matches()
{
	typedef std::pair<Frame*,int> FramePoint;
	std::map<FramePoint, std::list<PointTrack>::iterator> trackOfFramePoint;

	for(unsigned i=0; i<two_view_matches.size(); i++)
	{
		std::vector<PointMatch> &pml = two_view_matches[i].robust_matches;
		Frame* image0 = two_view_matches[i].frame[0];
		Frame* image1 = two_view_matches[i].frame[1];
		for(unsigned j=0; j<pml.size(); j++)
		{
			FramePoint point0(image0, pml[j].i0);
			std::list<PointTrack>::iterator track0;
			bool track0found = false;
			if(trackOfFramePoint.find(point0) != trackOfFramePoint.end()) {
				track0found = true;
				track0 = trackOfFramePoint[point0];
			}
			
			FramePoint point1(image1, pml[j].i1);
			std::list<PointTrack>::iterator track1;
			bool track1found = false;
			if(trackOfFramePoint.find(point1) != trackOfFramePoint.end()) {
				track1 = trackOfFramePoint[point1];
				track1found = true;
			}
			
			if(track0found && track1found)
			{
				//fusionem tracks
				// per cada punt del track1
				for(std::map<Frame*,int>::iterator iter = track1->track_indices.begin();
					iter != track1->track_indices.end();
					iter++ )
				{
					//l'afegim al track0
					track0->track_indices[iter->first] = iter->second;

					//i modifiquem el trackOfFramePoint corresponent
					trackOfFramePoint[FramePoint(iter->first,iter->second)] = track0;
				}
				
				//eliminem el track1
				tracks.erase(track1);
			}	
			else if(!track0found && track1found)
			{
				//afegim punt0 al track1
				track1->track_indices[image0] = pml[j].i0;
				trackOfFramePoint[point0] = track1;
			}
			else if(track0found && !track1found)
			{
				//afegim punt1 al track0
				track0->track_indices[image1] = pml[j].i1;
				trackOfFramePoint[point1] = track0;
			}
			else if(!track0found && !track1found)
			{
				//creem un nou track i mapem els dos punts
				PointTrack newTrack;
				newTrack.track_indices[image0] = pml[j].i0;
				newTrack.track_indices[image1] = pml[j].i1;
				tracks.push_back(newTrack);
				
				trackOfFramePoint[point0] = --tracks.end();
				trackOfFramePoint[point1] = --tracks.end();
			}
		}
	}
}

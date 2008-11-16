#include "mv.h"

namespace mv {


// Why the STL doesn't provide something like the below function by default, so
// that everyone doesn't invent their own, incompatible versions, is beyond me.
template<class T>
static void set_difference(const T &A, const T &B, T *C)
{
	std::set_difference(
			A.begin(), A.end(),
			B.begin(), B.end(),
			std::inserter(*C, C->begin()));
}

static size_t calculate_coverage(
		const TrackedSequence &ts,
		std::set<size_t> tracks,
	   	size_t frame_nr,
		int bins,
		int minimum_in_bin)
{

	size_t buckets[bins][bins];
	for (size_t i=0; i<bins; i++)
		for (size_t j=0; j<bins; j++)
			buckets[i][j] = 0;

	Frame *fr = ts[frame_nr];
	for (size_t i=0; i<fr->size(); i++) {
		PointFeature *pf = (PointFeature*)fr->at(i);
		size_t track_number = pf->get_track_number();
		if (tracks.count(track_number) == 1) {
			size_t x = (bins*(size_t)pf->x()) / ts.get_width();
			size_t y = (bins*(size_t)pf->y()) / ts.get_height();
//			assert(x >= 0);
//			assert(y >= 0);
			assert(x < bins);
			assert(y < bins);
			buckets[x][y]++;
		}
	}

	size_t coverage = 0;
	for (size_t i=0; i<bins; i++)
		for (size_t j=0; j<bins; j++)
			if (buckets[i][j] > minimum_in_bin)
				coverage++;

	return coverage;
}

void pick_keyframes(const TrackedSequence &ts, std::vector<size_t> *keyframes,
		Config *config)
{

	Config &config_ = *config;

	// Amount of screen coverage to tolerate before a new keyframe is made.
	PARAMETER(double, keyframe_coverage_loss_fraction, 0.93);
	// Minimum number of tracks we need to carry across
	PARAMETER(int, keyframe_min_tracks, 100);
	// Number of bins on a side
	PARAMETER(int, keyframe_bins, 4);
	// Need at least this many tracks in a bin to count it as 'full'
	PARAMETER(int, keyframe_minimum_in_bin, 8);

	bool found_keyframe = true;
	size_t last_keyframe = 0;
	size_t frame = 0;
	size_t last_keyframe_coverage;
	std::set<size_t> active_tracks;
	while (1) {
		Frame *fr = ts[frame];

		if (found_keyframe) {
			active_tracks.clear();
			for (size_t i=0; i<fr->size(); i++) {
				active_tracks.insert(fr->at(i)->get_track_number());
			}
			// Calculate this keyframe's score
			if (last_keyframe != frame)
				keyframes->push_back(last_keyframe);
			last_keyframe = frame;
			last_keyframe_coverage = calculate_coverage(
					ts, active_tracks, frame,
					keyframe_bins, keyframe_minimum_in_bin);
			found_keyframe = 0;
			fprintf(stderr, "found keyframe #%d=%d; new coverage=%d\n",
					keyframes->size(), frame, last_keyframe_coverage);
			frame++; // Since we need at least 2 frames separation
			frame++; // Since we need at least 2 frames separation
		}

		frame++;
		if (frame >= ts.size())
			break;
		fr = ts[frame];

		// Remove any tracks that got dropped from the last keyframe to this one
		std::set<size_t> frame_tracks;
		for (size_t i=0; i<fr->size(); i++) {
			frame_tracks.insert(fr->at(i)->get_track_number());
		}
		std::set<size_t> dropped;
		set_difference(active_tracks, frame_tracks, &dropped);
		std::set<size_t>::iterator it;
		for (it = dropped.begin(); it != dropped.end(); ++it) {
			active_tracks.erase(*it);
		}
		
		// Not enough tracks
		if (active_tracks.size() < keyframe_min_tracks) {
			found_keyframe = true;
			fprintf(stderr, "tracks go bye bye\n");
			continue;
		}
		
		// Lost too much coverage
		size_t coverage = calculate_coverage(
					ts, active_tracks, frame,
					keyframe_bins, keyframe_minimum_in_bin);
		fprintf(stderr, "frame=%d, coverage=%d\n", frame, coverage);
		if (coverage < last_keyframe_coverage*keyframe_coverage_loss_fraction) {
			found_keyframe = true;
			fprintf(stderr, "lost too much coverage\n");
		}
	}
	if (keyframes->size() == 0) {
		keyframes->push_back(0);
		assert(ts.size() > 0);
		keyframes->push_back(ts.size()-1);
	}

	std::sort(keyframes->begin(), keyframes->end());

	// KNOB
	if (ts.size() > 4 + keyframes->at(keyframes->size()-1) )
		keyframes->push_back(ts.size()-1);
}

} // namespace mv

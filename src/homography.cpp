#ifndef _HOMOGRAPHY_H
#define _HOMOGRAPHY_H

#include "mv.h"
#include "em_fit.h"
#include "homest.h"

using std::vector;

void homography_fit(const TrackedSequence &ts, int f1, int f2)
{
	assert (f1 < f2);
	Frame* frame1 = ts[f1];
	Frame* frame2 = ts[f2];
	vector<PointFeature*> track_f1;
	vector<PointFeature*> track_f2;

	/* Find tracks that are in both frames */
	foreach (Feature* feat, *frame1) {
		Track* track = feat->get_track();
		assert (track);
		foreach (Feature* ftrack, *track) {
			if (ftrack->get_frame() == frame2) {
				track_f1.push_back((PointFeature*)track);
				track_f2.push_back((PointFeature*)ftrack);
			}
		}
	}

	int num_matches = track_f1.size();
	double *xy1 = new double[2*num_matches];
	double *xy2 = new double[2*num_matches];
	for (int i=0; i<track_f1.size(); i++) {
		xy1[2*i+0] = track_f1[i]->x();
		xy1[2*i+1] = track_f1[i]->y();
		xy2[2*i+0] = track_f2[i]->x();
		xy2[2*i+1] = track_f2[i]->y();
	}

	double H[NUM_HPARAMS];
	homest(xy1, xy2, num_matches, 0.6, H, 1, 1, &number_outliers, 1/*verbose*/);
}

#endif // _HOMOGRAPHY_H

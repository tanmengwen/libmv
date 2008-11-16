#include <mv.h>
#include "resection.h"
#include "reconstruction.h"
#include "assert.h"

namespace mv {

// KNOB
const int min_points_to_resection = 100;

void resection_reconstruction(ProjectiveReconstruction *pr,
		double thresh,
		int max_new_cameras = -1)
{

	assert( pr->get_tracked_sequence());
	// FIXME this should be const&
	TrackedSequence *ts = pr->get_tracked_sequence();

	// Mapping between structure <-> track; remembering which tracks we've
	// already reconstructed.
	std::set<size_t> reconstructed_tracks;
	std::map<size_t,size_t> track_to_str;
	for (size_t i=0; i<pr->structure.size(); i++) {
		reconstructed_tracks.insert(pr->structure[i]->get_track_number());
		track_to_str[pr->structure[i]->get_track_number()] = i;
	}

	// Cache what frames we've already reconstructed
	std::set<size_t> reconstructed_cameras;
	for (size_t i=0; i<pr->cameras.size(); i++) {
		Frame *frame = pr->cameras[i]->get_frame();
		reconstructed_cameras.insert(frame->get_frame_number());
	}

	int added_cameras =  0;

	// now resection intermediate cameras
	// for each non f,m,l frame
	for (size_t i=0; i<ts->size(); i++) {

		if (reconstructed_cameras.count(i) == 1)
			continue;

		// Count how many features are in this frame
		int count = 0;
		for (size_t j=0; j<ts->at(i)->size(); j++) {
			Feature* feat = ts->at(i)->at(j);
			if (reconstructed_tracks.count(feat->get_track_number()) != 0)
				count++;
		}

		if (count < min_points_to_resection)
			continue;

		mat x(2, count); // Screen coordinates in this frame
		mat X(4, count); // Projective coordinates
		
		// Fill in x and X with relevant points
		int n=0;
		for (size_t j=0; j<ts->at(i)->size(); j++) {
			PointFeature* feat = (PointFeature*)ts->at(i)->at(j);
			size_t track_num = feat->get_track_number();
			if (reconstructed_tracks.count(feat->get_track_number()) == 0)
				continue;
			x(0,n) = feat->x();
			x(1,n) = feat->y();
			PointStructure* XX = (PointStructure*) pr->structure[track_to_str[track_num]];
			for (int k=0; k<4; k++) {
				X(k,n) = (*XX)[k];
			}
			n++;
		}

		mat P(3,4);
		double score = 100000;
		if (!robust_camera_resection(x, X, &P, &score)) {
			// this failed...
			continue;
		}
		printf("SCORE: %g\n", score);
		if (score > thresh) {
			// also failed
			printf("FAILED-----------------@thresh %g\n", thresh);
			continue;
		}

		ProjectiveCamera *pcam = new ProjectiveCamera;
		pcam->set_frame(ts->at(i));
		for (int j=0; j<3; j++)
			for (int k=0; k<4; k++)
				(*pcam)(j,k) = P(j,k);
		pr->push_camera(pcam);

		// Push measurements from this frame into the reconstruction; necessary
		// for bundle adjustment.
		// FIXME add a threshold; don't want to add insane errors
		for (size_t j=0; j<ts->at(i)->size(); j++) {
			PointFeature* feat = (PointFeature*)ts->at(i)->at(j);
			size_t track_num = feat->get_track_number();
			if (reconstructed_tracks.count(track_num) == 0)
				continue;
			Structure* XX = pr->structure[track_to_str[track_num]];

			// Check threshold. FIXME use an abstracted projection scheme...
			PointStructure *Xs = (PointStructure*)XX;
			vec xp = dot(*pcam, *Xs);
			xp /= xp[2];
			double ex = feat->x() - xp[0];
			double ey = feat->y() - xp[1];
			double error = ex*ex+ey*ey;
			if (sqrt(error) < thresh)
				pr->push_measurement(pcam, XX, feat);
		}
		added_cameras++;
		if (max_new_cameras != -1 && added_cameras >= max_new_cameras)
			break;
	}
	pr->number_items();
}

} // namespace mv

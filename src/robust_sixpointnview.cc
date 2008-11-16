#include "mv.h"
#include "ransac.h"
#include "nviewtriangulate.h"
#include "sixpointnview.h"
#include "/usr/include/valgrind/memcheck.h"

namespace mv {

// [camera i, point j] -> measurement
typedef std::map<std::pair<size_t,size_t>,PointFeature*> measurement_map;

class NViewSixPointRANSACDriver
	: public LMedSRANSACDriver<ProjectiveReconstruction*, size_t>
{
public:
	NViewSixPointRANSACDriver(mat tracks);
	void prepare_samples();
	void fit_subset(std::vector<size_t> &subset);
	void final_fit(std::vector<size_t> &subset) {
		(void) subset;
		if (last_reconstructed != best_fit) {
			last_reconstructed->delete_contents();
			delete last_reconstructed;
		}
		best_fit->bundle_adjust();
	}
	void calculate_residuals(ProjectiveReconstruction *&pr);
	void add_sample_to_best_fit(size_t track_num);
	virtual double calculate_error_threshold() {
		vec myresids(best_residuals);
		std::sort(myresids.begin(),myresids.end());
		// KNOB
		double median = 2*myresids[myresids.size()/2];
		dpmat(myresids);
//
		// If we were given measurements, then we want to ditch the structure
		// itself because it will be recomputed, and proper tracks will be
		// added.
		
		// bundle adjust the best fit before triangulating
//		if (nviews_ >= 18) {
//			best_fit->bundle_adjust();
//		}

		if (measurements != NULL) {
			printf("ditching measurements and structure.\n");
			for (size_t i=0; i<best_fit->structure.size(); i++) 
				delete best_fit->structure[i];
			best_fit->structure.clear();
			best_fit->measurements.clear();
			best_fit->cam_sees.clear();
			best_fit->visible_in.clear();

			// Link cameras to overall track frames (though not AVI frames)
			for (size_t i=0; i<nviews_; i++)
				 best_fit->cameras[i]->set_frame(
						 ((*measurements)[std::make_pair(i,0)])->get_frame());
		}

		return median;
	}
	void set_measurements(measurement_map *measurements) {
		this->measurements = measurements;
	}
private:
	mat tracks;
	size_t nviews_;
	size_t ntracks_;
	ProjectiveReconstruction *last_reconstructed;
	std::vector<mat*> final_Ps;
	measurement_map *measurements;
};

NViewSixPointRANSACDriver::NViewSixPointRANSACDriver(mat points)
{
	tracks = points;
	nviews_ = points.size2()/2;
	ntracks_ = points.size1();
	set_min_samples(6);
	for (size_t i=0; i<ntracks_; i++)
		all_samples.push_back(i);
	residuals.resize(all_samples.size());
	last_reconstructed = NULL;
	random_fits.resize(1);
	best_fit=NULL;
	measurements=NULL;
}

void NViewSixPointRANSACDriver::fit_subset(std::vector<size_t> &random_sample)
{
	mat points(3,nviews_*6);
	for (size_t i=0; i<nviews_; i++) {
		for (size_t j=0; j<6; j++) {
			points(0, 6*i+j) = tracks(random_sample[j], 2*i+0);
			points(1, 6*i+j) = tracks(random_sample[j], 2*i+1);
			points(2, 6*i+j) = 1.0;
		}
	}
	if (best_fit != last_reconstructed && last_reconstructed != NULL) {
		last_reconstructed->delete_contents();
		delete last_reconstructed;
	}
	last_reconstructed = six_point_n_view(points);
	random_fits[0] = last_reconstructed;

//	printf("fit samples=");
//	for (size_t j=0; j<6; j++) 
//		printf("%d,", random_sample[j]);
//	printf("\n");
//

//	if (nviews_ >= 18) {
//		double before_bundle = last_reconstructed->rms_reprojection_error();
//		last_reconstructed->bundle_adjust();
//		printf("RMS on random fit=%g\n", before_bundle);
//		printf("AFTER BUNDLE ADJU=%g\n",
//				last_reconstructed->rms_reprojection_error());
//	}
}

void NViewSixPointRANSACDriver::calculate_residuals(ProjectiveReconstruction *&fit)
{
	std::vector<mat*> Ps;
	for (size_t i=0; i<fit->cameras.size(); i++)
		Ps.push_back((ProjectiveCamera*)fit->cameras[i]);

	for (size_t i=0; i<ntracks_; i++) {
		mat points(nviews_, 2);
		for (size_t j=0; j<nviews_; j++) {
			points(j, 0) = tracks(i, 2*j+0);
			points(j, 1) = tracks(i, 2*j+1);
		}
		vec X_estimated;
		nviewtriangulate(points, Ps, &X_estimated);
		double total_error = 0;
//		printf("Entering error loop for i=%d with nviews_=%d\n",i,nviews_);
		for (size_t j=0; j<nviews_; j++) {
			vec x = dot(*Ps[j], X_estimated);
//			dpmat(x);
//			printf("normalized=\n");
//			double tmp = x[2];
			x[0] /= x[2];
			x[1] /= x[2];
			x[2] /= x[2];
//			dpmat(x);
			double dx = x[0] - points(j, 0);
			double dy = x[1] - points(j, 1);
//			printf("points(j=%d,0)=%g\n",j,points(j, 0));
//			printf("points(j=%d,1)=%g\n",j,points(j, 1));
			total_error += dx*dx + dy*dy;
		}
//		residuals[i] = sqrt(err);
//		if (total_error > 10)
//			printf("[%03d] WOAH bad error=%g\n",i, total_error);
//		else 
//			printf("[%03d]          error=%g\n",i, total_error);
		residuals[i] = total_error;
	} 
//	dpmat(residuals);
}

void NViewSixPointRANSACDriver::add_sample_to_best_fit(size_t sample)
{
	if (final_Ps.size() == 0) {
		for (size_t i=0; i<best_fit->cameras.size(); i++)
			final_Ps.push_back((ProjectiveCamera*)best_fit->cameras[i]);
	}

	mat points(nviews_, 2);
	for (size_t j=0; j<nviews_; j++) {
		points(j, 0) = tracks(sample, 2*j+0);
		points(j, 1) = tracks(sample, 2*j+1);
	}
	vec X_estimated;
	nviewtriangulate(points, final_Ps, &X_estimated);

	double total_error = 0;
	for (size_t j=0; j<nviews_; j++) {
		vec x = dot(*final_Ps[j], X_estimated);
		x /= x[2];
		double dx = x[0] - points(j, 0);
		double dy = x[1] - points(j, 1);
		total_error += dx*dx + dy*dy;
	}
	printf("Adding sample [%03d] with total error=%10g, residual=%10g\n",
			sample, total_error,best_residuals[sample]);
	printf("  RMS on fit so far fit=%g\n", best_fit->rms_reprojection_error());

	PointStructure *Xp = new PointStructure;
	Xp->assign(X_estimated);
	if (measurements != NULL) {
		PointFeature* pf = (*measurements)[std::make_pair(0,sample)];
		Xp->set_track(pf->get_track());
	}

	/* Push new structure */
	best_fit->push_structure(Xp);
	for (size_t j=0; j<nviews_; j++)  {
//		{
//			assert(best_fit->cameras[j] != NULL);
//			assert(Xp != NULL);
//			PointFeature *pf = new PointFeature(points(j, 0), points(j, 1));
//			assert(pf != NULL);
		if (measurements == NULL) {
			best_fit->push_measurement(
					best_fit->cameras[j], Xp, 
					new PointFeature(points(j, 0), points(j, 1)));
//			double err = best_fit.cameras[j]->reprojection_error(Xp, tft[j]);
//			printf("err[%d]=%g\n",j,sqrt(err));
		} else {
			best_fit->push_measurement(
					best_fit->cameras[j], Xp, 
					(*measurements)[std::make_pair(j,sample)]);
		}
	}
}

ProjectiveReconstruction *robust_sixpoint_n_view(
		mat &points, 
		double *error_thresh
		)
{
	NViewSixPointRANSACDriver driver(points);
	// KNOB
	if (!driver.search(50)) {
		return false;
	}
	ProjectiveReconstruction *pr = driver.best_fit;
	assert (driver.error_thresh_ > 0.0);
	*error_thresh = driver.error_thresh_;
	return pr;
}

// makes a map 
void make_measurement_map_between_frames(
		TrackedSequence &ts, size_t frame1, size_t frame2,
		mat *points_out,
		measurement_map *out)
{
	assert(out != NULL);
	measurement_map &tracks = *out;
	mat &points = *points_out;

	// go through each track and add it to the map
	std::vector<Track*> tracks_in_both_frames;

	// find the tracks that hit both frames and an ordering
	foreach (Feature* feat, *(ts[frame1])) {
		Track* track = feat->get_track();
		assert (track);
		foreach (Feature* ftrack, *track) {
			if (ftrack->get_frame_number() == (int)frame2) {
				tracks_in_both_frames.push_back(track);
				break;
			}
		}
	}

	size_t ntracks = tracks_in_both_frames.size();
	assert(frame1<frame2);
	size_t nviews = frame2-frame1+1;

	points.resize(ntracks,2*nviews);
	printf("ntracks=%d\n",ntracks);

	for (size_t j=0; j<tracks_in_both_frames.size(); j++) {
		Track *track = tracks_in_both_frames[j];
		size_t frame_offset_i = 0;
		for (size_t i=0; i<track->size(); i++) {
			Feature* ftrack = track->at(i);
			if (ftrack->get_frame_number() >= (int)frame1
					&& ftrack->get_frame_number() <= (int)frame2) {
				PointFeature *pf = (PointFeature*)ftrack;
				points(j,2*frame_offset_i+0) = pf->x();
				points(j,2*frame_offset_i+1) = pf->y();
//				VALGRIND_CHECK_VALUE_IS_DEFINED(
//					points(frame_offset_i,2*j+0));
//				VALGRIND_CHECK_VALUE_IS_DEFINED(
//					points(ntracks*frame_offset_i+j,1));
				assert(pf != NULL);
				tracks[std::make_pair(frame_offset_i, j)] = pf;
				frame_offset_i++;
			}
		}
		assert(frame_offset_i == nviews);
	}

	for (size_t i=0; i<points.size1(); i++)
		for (size_t j=0; j<points.size2(); j++)
				VALGRIND_CHECK_VALUE_IS_DEFINED(
					points(i,j));
}

ProjectiveReconstruction *robust_sixpoint_n_view_tracks(
		TrackedSequence &ts, size_t frame1, size_t frame2,
		size_t ransac_rounds,
		double *error_thresh
		)
{
	measurement_map measurements;
	mat points;

	make_measurement_map_between_frames(
			ts, frame1, frame2, &points, &measurements);

	NViewSixPointRANSACDriver driver(points);
	driver.set_measurements(&measurements);

	// KNOB
	if (!driver.search(ransac_rounds)) {
		return false;
	}

	ProjectiveReconstruction *pr = driver.best_fit;
	assert (driver.error_thresh_ > 0.0);
	*error_thresh = driver.error_thresh_;
	return pr;
}


} // namespace mv

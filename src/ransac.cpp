#include "mv.h"
#include "ransac.h"

extern "C" {
#include "lm.h"
}

namespace mv {

ThreeViewSixPointRANSACDriver::ThreeViewSixPointRANSACDriver(TrackedSequence &ts, size_t first, size_t middle, size_t last)
{
	this->frames[0] = ts[first];
	this->frames[1] = ts[middle];
	this->frames[2] = ts[last];

	prepare_samples();
}

ThreeViewSixPointRANSACDriver::ThreeViewSixPointRANSACDriver(TrackedSequence &ts, size_t first)
{
	this->frames[0] = ts[first];
	this->frames[1] = ts[first+1];
	this->frames[2] = ts[first+2];

	this->ts = &ts;

	prepare_samples();
}

ThreeViewSixPointRANSACDriver::ThreeViewSixPointRANSACDriver(Frame *frames[3])
{
	this->frames[0] = frames[0];
	this->frames[1] = frames[1];
	this->frames[2] = frames[2];

	prepare_samples();
}

void ThreeViewSixPointRANSACDriver::prepare_samples()
{
	/* For now assume frames are sorted and unique */
	assert (frames[0]->get_frame_number() < frames[1]->get_frame_number());
	assert (frames[1]->get_frame_number() < frames[2]->get_frame_number());

	/* Find tracks that are in all three frames */
	foreach (Feature* feat, *(frames[0])) {
		Track* track = feat->get_track();
		assert (track);
		int found = 1;
		ThreeFrameTrack tft;
		tft[0] = (PointFeature*)feat;

		foreach (Feature* ftrack, *track) {
			if (ftrack->get_frame() == frames[1] ||
			    ftrack->get_frame() == frames[2])
				tft[found++] = (PointFeature*) ftrack;
			if (found == 3)
				break;
		}
		if (found == 3)
			all_samples.push_back(tft);
	}

	/* Need 6 points in 3 views */
	set_min_samples(6);
//	assert (all_samples.size() >= min_samples);

	/* For now calculate error with replacement */
	residuals.resize(all_samples.size());
}

void ThreeViewSixPointRANSACDriver::fit_subset(std::vector<ThreeFrameTrack> &random_sample)
{
	random_fits.clear();
	six_point_three_view(random_sample, random_fits);
	for (size_t i=0; i<random_fits.size(); i++) {
		random_fits[i].set_tracked_sequence(ts);
	}
}

void ThreeViewSixPointRANSACDriver::calculate_residuals(ProjectiveReconstruction &fit)
{
	mat F;
	mat P1(3,4), P2(3,4), P3(3,4);
	P1.assign(*(ProjectiveCamera*)(fit.cameras[0]));
	P2.assign(*(ProjectiveCamera*)(fit.cameras[1]));

	find_fundamental_from_camera_matricies(P1, P2, F);
	int i=0;
	foreach (ThreeFrameTrack tft, all_samples) {
		vec x1, x2, x3;
		x1 = *tft[0];
		x2 = *tft[1];
		vec x1p, x2p;
//		find_optimal_points_given_fundamental(x1, x2, F, x1p, x2p);
		x1p.resize(3);
		x2p.resize(3);
		x1p.assign(x1);
		x2p.assign(x2);
		PointStructure X;
		X.assign(triangulate_dlt(P1, P2, x1p, x2p));
		double err = fit.cameras[2]->reprojection_error(&X, tft[2]);
		residuals[i] = sqrt(err);
		i++;
	} 
//	dpmat(residuals);
}

void ThreeViewSixPointRANSACDriver::add_sample_to_best_fit(ThreeFrameTrack tft)
{
	mat F;
	mat P1(3,4), P2(3,4), P3(3,4);
	P1.assign(*(ProjectiveCamera*)(best_fit.cameras[0]));
	P2.assign(*(ProjectiveCamera*)(best_fit.cameras[1]));

	find_fundamental_from_camera_matricies(P1, P2, F);
	vec x1, x2, x3;
	x1 = *(tft[0]);
	x2 = *(tft[1]);
	vec x1p, x2p;
	find_optimal_points_given_fundamental(x1, x2, F, x1p, x2p);
//	x1p.assign(x1);
//	x2p.assign(x2);

	// FIXME this is  a weird place to instantiate other structure...
	// really this should go elsewhere.
	PointStructure *Xp = new PointStructure(tft[0]->get_track());

	Xp->assign(triangulate_dlt(P1, P2, x1p, x2p));

	/* Push new structure */
	best_fit.push_structure(Xp);
	for (int j=0; j<3; j++)  {
		best_fit.push_measurement(
				best_fit.cameras[j], Xp, tft[j]);
//		double err = best_fit.cameras[j]->reprojection_error(Xp, tft[j]);
//		printf("err[%d]=%g\n",j,sqrt(err));
	}
}

void ThreeViewSixPointRANSACDriver::sample_is_not_in_best_fit(ThreeFrameTrack tft)
{
	(void) tft;
}

void ThreeViewSixPointRANSACDriver::final_fit(std::vector<ThreeFrameTrack> &subset)
{
	(void) subset;
}

//double ThreeViewSixPointRANSACDriver::score_fit(ProjectiveReconstruction &pr)
//{
//	(void) pr;
//	return 0;
//}


////////////////////////////////////////////////////////
// ProjectiveAlignmentRANSACDriver
////////////////////////////////////////////////////////

static void transform_cost_passthrough(double *p, double *hx, int m, int n, void *adata) {
	ProjectiveAlignmentRANSACDriver* pard =
		static_cast<ProjectiveAlignmentRANSACDriver*>(adata);
	pard->transform_cost(p,hx,m,n,adata); // do this so we can get at private data
}

void ProjectiveAlignmentRANSACDriver::transform_cost(double *p, double *hx, int m, int n, void *adata) {
	(void) n;
	(void) adata;
	// for nonlinear refinement
	mat H(4,4);
	fill(H, p); // confusing i know
	assert(m==16);
	mat Hinv = inv(H);
	mat PAHinv = dot(PA, Hinv);

	size_t pos = 0;
	foreach (PointPair pp, all_samples) {
		PointStructure *XA = pp.first;
		PointStructure *XB = pp.second;
		vec HXA = dot(H, *XA);
		MeasurementVisitor mv(B, XB->number);
		while (mv.next()) {
			assert (XB->number == mv.structure()->number);
			vec x = dot(*(ProjectiveCamera*)mv.camera(), HXA);
			x /= x[2];
			hx[pos++] = x[0];
			hx[pos++] = x[1];
		}
	} 
}

void ProjectiveAlignmentRANSACDriver::final_fit(std::vector<PointPair> &subset)
{
	(void) subset;
	double hh[16];

	// Calculate how many 'measurements' (aka points which live in
	// both reconstructions)
	int nmeasurements=0;
	foreach (PointPair pp, all_samples) {
		PointStructure *XB = pp.second;
		MeasurementVisitor mv(B, XB->number);
		while (mv.next()) {
			assert (XB->number == mv.structure()->number);
			nmeasurements++;
		}
	} 
	double *measurements = new double[nmeasurements*2];
	assert(measurements);

	// Fill in measurements
	size_t pos = 0;
	foreach (PointPair pp, all_samples) {
		PointStructure *XB = pp.second;
		MeasurementVisitor mv(B, XB->number);
		while (mv.next()) {
			assert (XB->number == mv.structure()->number);
			PointFeature *xp = (PointFeature*) mv.measurement();
			measurements[pos++] = xp->x();
			measurements[pos++] = xp->y();
		}
	} 

	// Fill hh with best estimate
	pos = 0;
	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			hh[pos++] = best_fit(i,j);

	fprintf(stderr, "DOING BEST FIT VIA DLEVMAR_DIF>>>>>>>>>>>>>>>>>>>>\n");
	calculate_residuals(best_fit);
	fprintf(stderr, "CURRENT SUM(residuals) = %g\n", sum(residuals)/residuals.size());

	dlevmar_dif(transform_cost_passthrough, hh, measurements, 16,nmeasurements*2,
			100/*maxits*/,
			NULL,NULL,NULL,NULL, /*don't care about details... */
			const_cast<ProjectiveAlignmentRANSACDriver*>(this));

	fill(best_fit, hh);

	fprintf(stderr, "DONE BEST FIT VIA DLEVMAR_DIF<<<<<<<<<<<<<<<<<<<<\n");
	calculate_residuals(best_fit);
	fprintf(stderr, "FINAL SUM(residuals) = %g\n", sum(residuals)/residuals.size());

	delete [] measurements;
}

ProjectiveAlignmentRANSACDriver::ProjectiveAlignmentRANSACDriver(
		ProjectiveReconstruction *A, 
		ProjectiveReconstruction *B)
{
	std::vector<size_t> common_A, common_B;
	A->common_cameras(*B, common_A, common_B);
	assert(common_A.size() == 1); // or maybe just use the first camera...

	PA = mat(*A->pcam(common_A[0]));
//	size_t thiscam_number = common_A[0];
	PB = mat(*B->pcam(common_B[0]));
	othercam_number = common_B[0];

	nullspace(PB, h);

	PBplusPA = dot(pinv(PB), PA);

	foreach (Structure* pts, A->structure) {

		// Only take points which are visible in the common frame
//		if (A->measurements.find(CToS(A->pcam(thiscam_number), pts))
//				== A->measurements.end())
//				continue;

		foreach (Structure* pts_other, B->structure) {
			// Points have to match (yes this is a dumb n^2 loop)
			if (pts->get_track_number() != pts_other->get_track_number())
				continue;

			// Reject points that are not in the common frame
//			if (B->measurements.find(CToS(B->pcam(othercam_number), pts_other))
//					== B->measurements.end())
//					continue;

			PointStructure* ptA = (PointStructure*) pts;
			PointStructure* ptB = (PointStructure*) pts_other;
			all_samples.push_back(PointPair(ptA, ptB));
		}
	}

	A->number_items();
	B->number_items();
	this->A = A;
	this->B = B;

	set_min_samples(5);

	/* For now calculate error with replacement */
	residuals.resize(all_samples.size());
	random_fits.resize(1);
}

void ProjectiveAlignmentRANSACDriver::fit_subset(std::vector<PointPair> &random_sample)
{
	std::vector<PointStructure*> XAs;
	std::vector<PointStructure*> XBs;
	for (size_t i=0; i<random_sample.size(); i++) {
		XAs.push_back(random_sample[i].first);
		XBs.push_back(random_sample[i].second);
	}
	random_fits[0] = mv::align_oneview(PBplusPA, h, XAs, XBs);
}

void ProjectiveAlignmentRANSACDriver::calculate_residuals(mat &H)
{
	int i=0;
	mat Hinv = inv(H);
	mat PAHinv = dot(PA, Hinv);

	std::vector<int> hits;
	hits.resize(residuals.size());

	// find H such that X_B = H*X_A
	foreach (PointPair pp, all_samples) {
		PointStructure *XA = pp.first;
		PointStructure *XB = pp.second;
		MeasurementVisitor mv(B, XB->number);
//		MeasurementVisitor mv(B);
		residuals[i] = 0;
		vec hx = dot(H, *XA);
		while (mv.next()) {
			assert (XB->number == mv.structure()->number);
//			if (mv.camera()->number != othercam_number)
//				continue;
			// FIXME abstract this projection code!!!!
			vec x = dot(*(ProjectiveCamera*)mv.camera(), hx);
			x /= x[2];
			PointFeature *x_measured = (PointFeature*) mv.measurement();
			double dx = x[0] - x_measured->x();
			double dy = x[1] - x_measured->y();
			double err = dx*dx+dy*dy;
			assert (err != 0.0);
			// FIXME This is _CLEARLY_ a mismatch with the two Rayleigh
			// distribution inlier/outlier model used for RANSAC. Still, it
			// works for now.
			residuals[i] += sqrt(err);
			hits[i]++;
		}
		i++;
	} 
	for (size_t i=0; i<hits.size(); i++) {
		assert (hits[i] > 0);
		residuals[i] /= hits[i];
	}
//	dpmat(residuals);
}

//void ProjectiveAlignmentRANSACDriver::add_sample_to_best_fit(PointPair p)
//{
//	inlier_samples.push_back(p);
//}

//void ProjectiveAlignmentRANSACDriver::sample_is_not_in_best_fit(PointPair p)
//{
	// This sample is an outlier, so we want to REMOVE it from
	// reconstruction A. This is a bit ugly, since it is an unexpected side
	// effect.
//	(void) p;
//}


} // namespace mv


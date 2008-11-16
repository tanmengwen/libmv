#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <set>
#include "cairo.h"
#include "mv.h"
#include "ransac.h"
#include "em_fit.h"
#include "sba.h"
#include "reconstruction.h"
#include "unscented.h"
#include "json.h"
#include "config.h"
#include "robust_sixpointnview.h"



/* TODO:
 *  - Add calls to number_items() at each site where numbering is broken; this
 *    will remove the spurrious calls to it.
 *
 */

extern "C" {
#include "pnmio.h"
//#include "extern/astrometry/cairoutils.h"
}

namespace mv {

////////////////////////////////////////////////////////////////////////////////
// Frame ///////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Frame::push_feature(Feature* feat)
{
	if (feat->get_frame() != this) 
		feat->set_frame(this);
	this->push_back(feat);
}

void Frame::push_feature(Track* track, Feature* feat)
{
	push_feature(feat);
	track->push_feature(feat);
	feat->set_track(track);
	feat->set_frame(this);
	feat->set_frame_number(get_frame_number());
	assert(feat->get_frame_number() == get_frame_number());
}

////////////////////////////////////////////////////////////////////////////////
// TrackedSequence /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void TrackedSequence::start_tracking(int cols, int rows)
{
	this->nrows = rows;
	this->ncols = cols;

	tc = KLTCreateTrackingContext();
	config_.parameter("window_size", 15, &tc->window_width);
	tc->window_height = tc->window_width;
	config_.parameter("minimum_feature_distance", 10, &tc->mindist);
	config_.parameter("affine_check", 0, &tc->affineConsistencyCheck);
	config_.parameter("num_features", 500, &nfeatures);
	fl = KLTCreateFeatureList(nfeatures);
	tc->sequentialMode = TRUE;

	assert(this->size() == 0);
}

void TrackedSequence::set_config(const Config &config) {
	config_ = config;
}

void TrackedSequence::drop_track(Track *track)
{
	std::vector<Track*>::iterator tloc = std::find(tracks.begin(), tracks.end(), track);
	tracks.erase(tloc);

	foreach (Feature* feat, *track) {
		Frame &f = *(feat->get_frame());
		Frame::iterator floc = std::find(f.begin(), f.end(), feat);
		assert(floc != f.end());
		f.erase(floc);
		delete feat;
	}

	delete track;
}

bool TrackedSequence::track_sigma_points_and_compute_sigma(
		PointFeature *pf_previous,
		PointFeature *pf,
		UnscentedTransform *ut)
{
	assert (ut);
	bool failed = false;
	for (size_t i=0; i<ut->samples(); i++) {
		KLT_FeatureRec klt_feat;
		memset(&klt_feat, 0, sizeof(KLT_FeatureRec));
		klt_feat.x = ut->sample(i)[0];
		klt_feat.y = ut->sample(i)[1];
		// printf("tracking i=%d,x=%10g,y=%10g\n",i,klt_feat.x,klt_feat.y);
		KLTTrackOneFeature(tc, &klt_feat);
		if (klt_feat.val < 0) {
			printf("Dropped a feature! x=%g,y=%g\n", klt_feat.x, klt_feat.y);
			dpmat(pf_previous->sigma);
			failed = true;
			break;
		}
		ut->sample(i)[0] = klt_feat.x;
		ut->sample(i)[1] = klt_feat.y;
	}
	if (!failed) {
		ut->compute_transformed_moments();
		fill(pf->sigma, ut->covariance());
	} 
	return failed;
}

void TrackedSequence::track_and_replace_features(unsigned char *image)
{
	if (!started) {
		KLTSelectGoodFeatures(tc, image, ncols, nrows, fl);
		started = true;
		recent_tracks.resize(nfeatures);
		for (int i=0; i<nfeatures; i++)
			recent_tracks[i] = NULL;
	} else {
		KLTTrackFeatures(tc, last, image, ncols, nrows, fl);
		PARAMETER(bool, use_bucketing_replacement, true);
		if (use_bucketing_replacement) {
			// FIXME move the bucket size variables so they are taken from
			// tc->, then we can make them knobs that get stored.
			KLTReplaceLostFeaturesBuckets(tc, fl);
		} else {
			PARAMETER(double, replacement_threshold, 0.7);
			if (KLTCountRemainingFeatures(fl)
					< replacement_threshold*this->nfeatures)
				KLTReplaceLostFeatures(tc, image, ncols, nrows, fl);
		}
	}
}

// FIXME this is nasty. What this needs is to abstract the underlying 'tracker'
// and 'tracker data' so that we could (for example) integrate the OpenCV
// tracker easily. Need to integrate a JSON configuration system!
void TrackedSequence::push_frame(unsigned char *image, string filename,
		int orig_frame_ind)
{
	track_and_replace_features(image);

	Frame *f = new Frame();
	f->set_frame_number(size());
	f->set_orig_ind(orig_frame_ind);
	f->set_filename(filename);
	push_back(f);

	PARAMETER(double, initial_point_sigma2, 1.0);
	PARAMETER(double, alpha, 0.9);
	PARAMETER(double, beta, 2.0); // always 2.0 for gaussians
	PARAMETER(double, kappa, 0.0); // the use 0.0
	PARAMETER(bool,   do_unscented_transform, true);
	PARAMETER(bool,   reuse_unscented_samples, true);
	PARAMETER(bool,   display_mean_change, false);

	for (int feat=0; feat<fl->nFeatures; feat++)  {
		KLT_Feature ft = fl->feature[feat];
		if (ft->val > 0) { 
			// val > 0 ==> new feature, val is the hessians min eigenvalue
			PointTrack *pt = new PointTrack();
			assert (pt != NULL);
			pt->set_number(tracks.size());
			tracks.push_back(pt);
			recent_tracks[feat] = pt;
		} else if (ft->val < 0) { 
			// val < 0 ==> tracking failure
			recent_tracks[feat] = NULL;
			// FIXME leaking UT attached to pf's.
		}

		if (!recent_tracks[feat]) {
			continue; // in case we don't start out initialized
		}

		PointFeature* pf = new PointFeature(ft->x, ft->y);
		Track *pt = recent_tracks[feat];

		if (!do_unscented_transform) {
			f->push_feature(pt, pf);
			continue;
		}

		assert (pf != NULL);
		// Now push the sigma points through tracking
		if (pt->size() == 0) {
			// new feature, set sigma directly
			pf->sigma = initial_point_sigma2*eye(2,2);

//			if (reuse_unscented_samples) {
				// these samples are kept around during tracking instead of
				// being recomputed at each step. In theory, this should
				// increase the accuracy of the covariance estimate.
				double mean[2] = {pf->x(), pf->y()};
				pf->ut = new UnscentedTransform(
						mean, pf->sigma.data().begin(),
						2, kappa, alpha, beta);
//				printf("samples generated=\n");
//				for (size_t i=0; i<pf->ut->samples(); i++) {
//					printf("   x=%10g, y=%10g\n",
//						pf->ut->sample(i)[0],
//						pf->ut->sample(i)[1]);
//				}
				assert(pf->ut);
//			}
			f->push_feature(pt, pf);
		} else {
			size_t end = pt->size()-1;
			PointFeature *pf_previous =(PointFeature*) pt->at(end);

			bool lost;
			if (reuse_unscented_samples) {
				lost = track_sigma_points_and_compute_sigma(
						pf_previous, pf, pf_previous->ut);
				ft->x = pf_previous->ut->mean()[0];
				ft->y = pf_previous->ut->mean()[1];
			} else {
//				double mean[2] = {pf_previous->x(), pf_previous->y()};
				assert(pf->ut == NULL);
				pf->ut = new UnscentedTransform(
						pf_previous->data().begin(),
						pf_previous->sigma.data().begin(),
						2, kappa, alpha, beta);
				lost = track_sigma_points_and_compute_sigma(
						pf_previous, pf, pf->ut);
				ft->x = pf->ut->mean()[0];
				ft->y = pf->ut->mean()[1];
			}

			if (lost) {
				// one of the points failed to track; point is garbage
				recent_tracks[feat] = NULL;
				KLTClearFeature(ft);
				if (reuse_unscented_samples)
					delete pf_previous->ut;
				delete pf;
			} else {
				// Set the feature to have the mean of the transform, rather
				// than the one it started with.
				pf->set(ft->x, ft->y);
				if (reuse_unscented_samples)
					pf->ut = pf_previous->ut;
				f->push_feature(pt, pf);
			}
		}
	}

	// FIXME this should be in a function, yikes!
	if (1) {
		PARAMETER(string, debug_file_pattern, "debug%08d.png");
		cairo_surface_t *surface;
		cairo_t *cr;

		surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, ncols, nrows);
		cr = cairo_create (surface);
		unsigned char *im = cairo_image_surface_get_data(surface);
		int stride = cairo_image_surface_get_stride(surface);
		/* copy image into the surface */
		memset(im, 0, nrows*ncols*sizeof(int));
		for (int i=0; i<nrows; i++) {
			for (int j=0; j<ncols; j++) {
				im[i*stride + 4*j + 0] = image[i*ncols + j];
				im[i*stride + 4*j + 1] = image[i*ncols + j];
				im[i*stride + 4*j + 2] = image[i*ncols + j];
				im[i*stride + 4*j + 3] = 0xff;
			}
		}

		PARAMETER(int,    cairo_tail_length, 10);
		PARAMETER(double, cairo_tail_start_opacity, 0.5);

		float half_w = tc->window_width/2.;
		float half_h = tc->window_height/2.;
		for (int feat=0; feat<fl->nFeatures; feat++)  {
			KLT_Feature ft = fl->feature[feat];
			if (ft->val < 0) {
				continue;
			}

			size_t sz = recent_tracks[feat]->size();
			// first draw trailer lines; we want them underneath
			if (sz > 1) {
				size_t current = sz-2;
				// FIXME config params!
				float r=236./256.;
				float g=136./256.;
				float b=16./256.;
				cairo_set_source_rgba(cr, r,g,b, 1.0);
				cairo_move_to(cr, ft->x, ft->y);
				for (int k=0; k<cairo_tail_length && current > 0; k++, current--) {
					// draw a line connecting past feature locations
					PointFeature *pff1 = (PointFeature *)recent_tracks[feat]->at(current+1);
					PointFeature *pff2 = (PointFeature *)recent_tracks[feat]->at(current);
					cairo_move_to(cr, pff1->x(), pff1->y());
					cairo_line_to(cr, pff2->x(), pff2->y());
					cairo_set_source_rgba(cr, r,g,b, 
							cairo_tail_start_opacity
							*(cairo_tail_length - k)/(float)cairo_tail_length);
					cairo_stroke(cr);
				}
			}

			// then draw the box around the pixels KLT is following
			if (sz > 1) {
				cairo_set_source_rgba(cr, 1.0, 0.0, 1.0, 0.5);
			} else {
				cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.5);
			}
			cairo_rectangle(cr, ft->x-half_w, ft->y-half_h,
					tc->window_width,
					tc->window_height);
			cairo_set_line_width (cr, 1.4);
			cairo_stroke(cr);

			// then draw the uncertainty ellipsoid
			assert( (recent_tracks[feat]) );
			if (do_unscented_transform) {
				PointFeature *pf_current = (PointFeature *)recent_tracks[feat]->at(sz-1);
				mat transf(2,2);
				sqrtm(pf_current->sigma, &transf);
				cairo_matrix_t ellipse;
				cairo_matrix_init(&ellipse, transf(0,0), transf(1,0), transf(0,1),
						transf(1,1), pf_current->x(), pf_current->y()); 
				cairo_save(cr);
				cairo_transform(cr, &ellipse);
//				cairo_arc(cr, 0., 0., tc->window_height/2.0,
				cairo_arc(cr, 0., 0., initial_point_sigma2,
						0., 2 * 3.1415926535897931);
				cairo_restore(cr);
				if (sz > 1) {
					cairo_set_source_rgba(cr, 0.0, 1.0, 0.0, 1.0);
				} else {
					cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.6);
				}
				cairo_stroke(cr);
				if (sz > 1) {
					if (reuse_unscented_samples) {
						cairo_set_source_rgba(cr, 1.0, 1.0, 0.0, 1.0);
					} else {
						cairo_set_source_rgba(cr, 0.0, 1.0, 1.0, 1.0);
					}
				}
				UnscentedTransform *ut = pf_current->ut;
				for (size_t i=0; i<ut->samples(); i++) {
					cairo_arc(cr, ut->sample(i)[0], ut->sample(i)[1], 0.5,
							0., 2 * 3.1415926535897931);
					cairo_stroke(cr);
				}
			}
		}

		cairo_destroy(cr);
		char fnameout[100];
		sprintf(fnameout, "cairo%04d.png", size());
		cairo_surface_write_to_png(surface, fnameout);
		cairo_surface_destroy(surface);
	}

	last = image;
}

void TrackedSequence::stop_tracking()
{
	// Drop any length-1 tracks
	int deleted = 1;
	while (deleted) {
		deleted = 0;
		foreach (Track *track, tracks) {
			if (track->size() == 1) {
				drop_track(track);
				deleted = 1;
				break;
			}
		}
	}
	started = false;
}

void TrackedSequence::track_pgm_sequence(const char* format, int start, int end)
{
	char fnamein[100];
	unsigned char *img1, *img2;

	set_format(format);

	sprintf(fnamein, format, start);
	img1 = pgmReadFile(fnamein, NULL, &ncols, &nrows);
	img2 = (unsigned char *) malloc(ncols*nrows*sizeof(unsigned char));

	start_tracking(ncols, nrows);
	push_frame(img1, fnamein, start);

	for (int i=start+1; i<=end; i++)  {
		sprintf(fnamein, format, i);
		pgmReadFile(fnamein, img2, &ncols, &nrows);
		push_frame(img2, fnamein, i);
	}

	stop_tracking();
}

void TrackedSequence::number_items()
{
	int frnum = 0;
	foreach (Frame* fr, *this) {
		size_t i=0; 
		foreach (Feature* ft, *fr) {
			ft->set_number(i++);
			ft->set_frame_number(frnum);
		}
		assert (frnum == fr->get_frame_number());
		frnum++;
	}
	for (size_t i=0; i<tracks.size(); i++) {
		tracks[i]->set_number(i);
	}
}


///////////////////////////////////////////////////////////////////////////////
// ProjectiveCamera    ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

double ProjectiveCamera::reprojection_error(Structure* s, Feature *f)
{
	assert (s->get_type() == POINT);
	PointStructure *X = (PointStructure*) s;
	// FIXME assert feature type
	PointFeature *x = (PointFeature*) f;

	vec y = dot(*this, *X);
	y /= y[2];

	//	dpmat(*x);
	//	dpmat(y);
	//	dpmat(*X);

	double dx = x->x() - y[0];
	double dy = x->y() - y[1];
	return dx*dx + dy*dy;
}

PointFeature ProjectiveCamera::project_point(PointStructure *X)
{
	PointFeature res;
	res.assign(dot(*this,*X));
	res /= res[2];
	return res;
}

Camera* ProjectiveCamera::copy()
{
	ProjectiveCamera* pc = new ProjectiveCamera();
	pc->assign(*this);
	copy_into(pc);
	return pc;
}

///////////////////////////////////////////////////////////////////////////////
// PointStructure    ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

Structure* PointStructure::copy()
{
	PointStructure* ps = new PointStructure();
	ps->assign(*this);
	copy_into(ps);
	return ps;
}

////////////////////////////////////////////////////////////////////////////////
// ProjectiveReconstruction 
////////////////////////////////////////////////////////////////////////////////

void ProjectiveReconstruction::estimate_errors()
{
	size_t n = structure.size();
	vec sse(n);
	vec nsamps(n);
	fill(sse, 0.);
	fill(nsamps, 0.);
	number_items();
	sigma2 = 0.;

	MeasurementVisitor mv(this);
	while (mv.next()) {
		PointStructure* ss = (PointStructure*) mv.structure();
		ProjectiveCamera* cam = (ProjectiveCamera*) mv.camera();
		PointFeature* f = (PointFeature*) mv.measurement();
		PointFeature fp = cam->project_point(ss);
		double dx = f->x() - fp.x();
		double dy = f->y() - fp.y();
		double residual2 = dx*dx + dy*dy;
		sse[mv.structure()->number] += residual2;
		nsamps[mv.structure()->number] += 1.;
		sigma2 += dx*dx + dy*dy;
	}
	sigma2 /= 2*n;

	for (size_t i=0; i<n; i++)
		pstr(i)->sigma2 = sse[i]/2/nsamps[i];
}

// Someday this should be lifted to the Reconstruction class, but for now this
// will do. Also, this is using the really dumb method of just merging
// sequentially rather than hierarchicaly. 
void ProjectiveReconstruction::reconstruct(TrackedSequence &ts, size_t ransac_rounds)
{
	assert(ts.size() >= 3);

	for (size_t i=0; i+2<ts.size(); i+=2) {
		printf("i=%d\n",i);
		ThreeViewSixPointRANSACDriver drv(ts, i);
		if (!drv.search(ransac_rounds)) {
			printf("Failed at reconstruction %d\n", i);
			return;
		}
		if (i != 0) 
			robust_oneview_align(drv.best_fit);

		merge(drv.best_fit);
		printf("BEFORE BUNDLE rms error=%g i=%d\n",rms_reprojection_error(), i);
		printf("BEFORE BUNDLE mean reprojection error=%g i=%d\n",mean_reprojection_error(), i);
		if (rms_reprojection_error() > 2)
			assert(0);
		bundle_adjust();
		printf("AFTER BUNDLE rms error=%g i=%d\n",rms_reprojection_error(), i);
		printf("AFTER BUNDLE mean reprojection error=%g i=%d\n",mean_reprojection_error(), i);
	}
}

bool ProjectiveReconstruction::robust_oneview_align(ProjectiveReconstruction &pr)
{
	// FIXME add outlier pruning here
	ProjectiveAlignmentRANSACDriver drv(this, &pr);
	if (drv.search(300)) {
		transform(drv.best_fit);

		// Now use 'regular' LM to minimize the error over the transform

		return true;
	} else {
		return false;
	}
}

void ProjectiveReconstruction::print_cameras()
{
	printf("Ps = []\n");
	for (size_t i=0; i<cameras.size(); i++) {
		printf("Ps.append(");
		pnumpy(*pcam(i));
		printf(")\n");
	}
}

void ProjectiveReconstruction::print_structure()
{
	printf("Ss = []\n");
	for (size_t i=0; i<structure.size(); i++) {
		printf("Ss.append(");
		pnumpy(*pstr(i));
		printf(")\n");
	}
}

void ProjectiveReconstruction::normalize_camera(int num)
{
	mat H = eye(4);
	subrange(H, 0,3, 0,4) = *pcam(num);

	dpmat(H);
	transform(H);
}

// Shrink xp towards x such that (x-new_xp)^2 becomes huber in xp, needed
// because sba doesn't inherently have a robust cost equivalent.  The +/-
// conditional is to keep xp on the right side of x.
double huber_shrink(double x, double xp, double B) {
	double e = x - xp;
	double fe = fabs(e);
	if (fe < B)
		return xp;
	double shrink = sqrt(2*B*fe - B*B);
	if (x < xp)
		// CHECKING; commented out is what i *think* is right; but sign may be
		// backwards so checking
		return x + shrink;
	return x - shrink;
//		return x - shrink;
//	return x + shrink;
}

// really this is part of ProjectiveReconstruction
void sba_proj_projective(int jj, int ii, double *aj, double *bi, double *xij, void *adata)
{
	/* Don't generate compile warnings */
	(void) ii;
	(void) jj;
	(void) adata;

	// Convert bi into 4-vector P3 point
	vec bi_(3);
	fill(bi_, bi);
	//	double bimag = calc_PN_from_RN(bi, X, 4);
	vec X = disk_to_sphere(bi_);
	double bimag = norm_2(X);

	// Convert aj into 12-vector projection matrix
	//	double P[12];
	//	double ajmag = calc_PN_from_RN(aj, P, 12);
	vec aj_(11);
	fill(aj_, aj);
	vec P = disk_to_sphere(aj_);
	double ajmag = norm_2(P);

	// for now... later on wrap these across the sphere instead of
	// crashing!
	//	assert (bimag < M_PI + 1e-7);
	//	assert (ajmag < M_PI + 1e-7);
	assert (bimag < M_PI + 1.);
	assert (ajmag < M_PI + 1.);

	// then project
	double PzTX = 0.0;
	xij[0] = 0.0;
	xij[1] = 0.0;
	int i;
	for (i=0; i<4; i++) {
		xij[0] += X[i]*P[0*4 + i];
		xij[1] += X[i]*P[1*4 + i];
		PzTX   += X[i]*P[2*4 + i];
	}
	xij[0] /= PzTX;
	xij[1] /= PzTX;

	// Now make it a 'robust' error to not penalize outliers too much
	ProjectiveReconstruction *pr = (ProjectiveReconstruction*)adata;
	PointFeature* pf = (PointFeature*)pr->get_feat(jj,ii);

	// Use a cost that reflects our uncertainty; i.e. errors on points we are
	// not confident about are not heavily penalized.
	if (pr->use_mahalanobis_distance_) {
		// Note that we need to matrix-multiply by the inverse square root of
		// covariance, because we don't explicitly compute the squared error; we
		// only compute the components.
		// ... compute xij[0,1] = dot(sqrtm(inv(covariance)), xij[0,1])
		vec xx(2);
		xx(0) = xij[0] - pf->x();
		xx(1) = xij[1] - pf->y();
		vec yy = dot(pf->sigma_inv_sqrt, xx);
		xij[0] = yy(0) + pf->x();
		xij[1] = yy(1) + pf->y();
	}

	if (pr->use_huber_cost_) {
		xij[0] = huber_shrink(pf->x(), xij[0], pr->huber_threshold_);
		xij[1] = huber_shrink(pf->y(), xij[1], pr->huber_threshold_);
	}
}


// order by structure number, then cam number
// as required by SBA
bool compare_ctos(const CToS &a, const CToS &b) 
{
	if (a.second->get_number() < b.second->get_number())
		return true;
	else if (a.second->get_number() == b.second->get_number())
		return a.first->get_number() < b.first->get_number();
	return false;
}

void ProjectiveReconstruction::bundle_adjust()
{
	int n=structure.size();  // number of 3d pts
	int m=cameras.size();    // number of cameras
	int mcon=0; // number of cameras to keep fixed
	int cnp=11; // number of params per cam (11 -> 12 homogenous)
	int pnp=3;  // number of params per point (3 -> 4 homogenous)
	int mnp=2;  // number of params per image measurement (how can this not be 2)

	size_t visible_2d_points = 0;
	visible_2d_points = measurements.size();

	number_items();

	char* vmask = new char[n*m];
	for (int i=0; i<n*m; i++)
		vmask[i] = 0;

	double *xij = new double[visible_2d_points*2];
	size_t ptij = 0;

	std::vector<CToS> pairs;
	MeasurementVisitor mv(this);
	while (mv.next()) {
		pairs.push_back(std::make_pair(mv.camera(), mv.structure()));
	}
	std::sort(pairs.begin(), pairs.end(), compare_ctos);

	PARAMETER(bool, projective_bundle_use_mahalanobis_distance, false);
	use_mahalanobis_distance_ = projective_bundle_use_mahalanobis_distance;
	mat tmp(2,2);
	foreach(CToS ctos, pairs) {
		PointStructure* ss = (PointStructure*) ctos.second;
		ProjectiveCamera* cam = (ProjectiveCamera*) ctos.first;
		PointFeature* f = (PointFeature*) measurements[ctos];
		if (use_mahalanobis_distance_) {
			f->sigma_inv_sqrt = f->sigma;
			inv2inplace(&f->sigma_inv_sqrt);
			sqrtm(f->sigma_inv_sqrt, &f->sigma_inv_sqrt);
		}
		xij[ptij++] = f->x();
		xij[ptij++] = f->y();
		vmask[ss->number*m + cam->number] = 1;
		//		if (ptij < 200)
		//		printf("ss=%d,cam=%d\n", ss->number,cam->number);
	}

	/* Pack the cameras and structure */
	int i = 0;
	double *p = new double[m*cnp + n*pnp];
	foreach (Camera* cam, cameras) {
		ProjectiveCamera* pr = (ProjectiveCamera*) cam;
		mat P(3,4);
		P.assign(*pr);
		vec pp = sphere_to_disk(ravel(P));
		assert(pp.size()==11);
		for (int j=0; j<11; j++)
			p[i++] = pp[j];
	}
	foreach (Structure* s, structure) {
		PointStructure* ps = (PointStructure*) s;
		vec X(4);
		X.assign(*ps);
		vec x = sphere_to_disk(X);
		assert(x.size()==3);
		for (int j=0; j<3; j++)
			p[i++] = x[j];
	}

	double info[10];
	double opts[4];
	PARAMETER(double, projective_bundle_init_mu, 1e-3);
	PARAMETER(double, projective_bundle_termination_threshold, 1e-9);
	opts[0]=projective_bundle_init_mu;
	opts[1]=projective_bundle_termination_threshold;
	opts[2]=projective_bundle_termination_threshold;
	opts[3]=projective_bundle_termination_threshold;

	PARAMETER(bool, projective_bundle_use_huber_cost, true);
	use_huber_cost_ = projective_bundle_use_huber_cost;
	if (projective_bundle_use_huber_cost) {
		PARAMETER(double, projective_bundle_huber_threshold, 3.0);
		huber_threshold_ = projective_bundle_huber_threshold;
	}

	PARAMETER(int, projective_bundle_max_iterations, 30);
	int ret = sba_motstr_levmar(n, m, mcon, vmask, p, cnp, pnp, xij, mnp,
			sba_proj_projective,
			NULL,     /* numeric jacobian */
			//			sba_projjac_projective, /* analytic jacobian!!! */
			this,     /* extra parameter */
			projective_bundle_max_iterations, /* max iter */
			3,        /* be verbose*/ 
			opts,     /* options */
			info);

	printf("INFO:\n");
	for (i=0; i<10; i++)
		printf("%d - %g\n", i, info[i]);
	printf("RET: %d\n",ret);

	/* Unpack the cameras and structure */
	i = 0;
	foreach (Camera* cam, cameras) {
		ProjectiveCamera* pr = (ProjectiveCamera*) cam;
		vec p11(11);
		fill(p11, p+i);
		vec P = disk_to_sphere(p11);
		for (int j=0; j<3; j++) 
			for (int k=0; k<4; k++) 
				(*pr)(j,k) = P[4*j+k];
		i += 11;
	}
	foreach (Structure* s, structure) {
		vec x3(3);
		fill(x3, p+i);
		vec X = disk_to_sphere(x3);
		PointStructure* ps = (PointStructure*) s;
		for (int j=0; j<4; j++) 
			(*ps)[j] = X[j];
		i += 3;
	}
}

mat ProjectiveReconstruction::align_oneview(ProjectiveReconstruction &other)
{
	std::vector<size_t> common_this, common_other;
	common_cameras(other, common_this, common_other);
	assert(common_this.size() == 1); // or maybe just use the first camera...


	mat PsA(3,4);
	mat PsB(3,4);
	PsA.assign(*pcam(common_this[0]));
	PsB.assign(*other.pcam(common_other[0]));

	vec h;
	nullspace(PsB, h);

	mat PBplusPA = dot(pinv(PsB), PsA);

	std::vector<PointStructure*> XAs;
	std::vector<PointStructure*> XBs;
	foreach (Structure* pts, structure) {
		foreach (Structure* pts_other, other.structure) {
			if (pts->get_track_number() != pts_other->get_track_number())
				continue;

			PointStructure* pt = (PointStructure*) pts;
			PointStructure* pt_other = (PointStructure*) pts_other;

			XAs.push_back(pt);
			XBs.push_back(pt_other);
		}
	}
	return mv::align_oneview(PBplusPA, h, XAs, XBs);
}

void ProjectiveReconstruction::transform(mat &H)
{
	/* Transfrom points in A to align with B */
	/* HX_a = X_b */
	foreach (Structure* pts, structure) {
		PointStructure* pt = (PointStructure*) pts;
		vec tmp = dot(H,*pt); // THESE TEMPORARIES ARE NECESSARY! ublas misses the aliasing
		pt->assign(tmp);
	}

	/* P_b = P_a*Hinv */
	mat Hinv = inv(H);
	dpmat(Hinv);
	foreach (Camera* cam, cameras) {
		ProjectiveCamera* pc = (ProjectiveCamera*) cam;
		mat tmp = dot(*pc, Hinv);
		pc->assign(tmp);
	}
}

Reconstruction *ProjectiveReconstruction::copy()
{
	ProjectiveReconstruction *pr = new ProjectiveReconstruction();
	pr->sigma2 = sigma2;
	copy_into(pr);
	return pr;
}

////////////////////////////////////////////////////////////////////////////////
// Reconstruction 
////////////////////////////////////////////////////////////////////////////////

Reconstruction *Reconstruction::copy_into(Reconstruction* nr)
{
	std::map<Structure*,Structure*> s2s;
	foreach (Structure* st, structure) {
		Structure* new_st = st->copy();
		assert(new_st != st);
		s2s[st] = new_st;
		nr->push_structure(new_st);
	}

	std::map<Camera*,Camera*> c2c;
	foreach (Camera* cam, cameras) {
		Camera* new_cam = cam->copy();
		c2c[cam] = new_cam;
		nr->push_camera(new_cam);

		//		PointStructure xxx;
		//		xxx[0] = 1;
		//		xxx[1] = 1;
		//		xxx[2] = 1;
		//		xxx[3] = 1;
		//		PointFeature xx = ((ProjectiveCamera*)new_cam)->project_point(&xxx);
		//		printf("measuremenpre_add->x()=%g\n", xx.x());
		//		printf("measuremenpre_add->y()=%g\n", xx.y());
		//		printf("NEW CAMERA=%p, old=%p\n",new_cam, cam);
	}

	MeasurementVisitor mv(this);
	while (mv.next()) {
		assert(c2c.count(mv.camera()) == 1);
		assert(s2s.count(mv.structure()) == 1);
		PointFeature xx = ((ProjectiveCamera*)c2c[mv.camera()])->project_point((PointStructure*)s2s[mv.structure()]);
		//		printf("measurement->x()=%g\n", xx.x());
		//		printf("measurement->y()=%g\n", xx.y());
		nr->push_measurement(
				c2c[mv.camera()],
				s2s[mv.structure()],
				mv.measurement());
		//		PointFeature* ps = (PointFeature*)mv.measurement();
	}

	nr->set_tracked_sequence(this->get_tracked_sequence());

	assert(structure.size() == nr->structure.size());
	assert(cameras.size() == nr->cameras.size());
	assert(cam_sees.size() == nr->cam_sees.size());
	assert(visible_in.size() == nr->visible_in.size());

	return nr;
}

double Reconstruction::rms_reprojection_error()
{
	double total_error = 0.;
	size_t nmeasurements = 0;
	if (measurements.size() == 0u)
		return 0.0;
	MeasurementVisitor mv(this);
	while (mv.next()) {
//		printf("cam=%p,str=%p,measurement=%p\n",
//				mv.camera(),
//				mv.structure(),
//				mv.measurement());

		assert(mv.camera() != NULL);
		assert(mv.structure() != NULL);
		assert(mv.measurement() != NULL);
		double err = mv.camera()->reprojection_error(
				mv.structure(),
				mv.measurement());
		total_error += err;
		nmeasurements++;
	}
	return sqrt(total_error/nmeasurements);
}

void Reconstruction::print_residuals()
{
	size_t i=0;
	vec resids(cam_sees.size());
	MeasurementVisitor mv(this);
	while (mv.next()) {
		double err = mv.camera()->reprojection_error(
				mv.structure(),
				mv.measurement());
		resids[i++] = sqrt(err);
	}
	printf("# ALL residuals for this reconstruction\n");
	dpmat(resids);
}

double Reconstruction::mean_reprojection_error()
{
	double total_error = 0.;
	int nmeasurements = 0;
	MeasurementVisitor mv(this);
	while (mv.next()) {
		double err = mv.camera()->reprojection_error(
				mv.structure(),
				mv.measurement());
		total_error += sqrt(err);
		nmeasurements++;
	}
	return total_error/nmeasurements;
}

void Reconstruction::common_cameras(Reconstruction &other,
		std::vector<size_t> &common_this,
		std::vector<size_t> &common_other)
{
	number_items();
	other.number_items();

	foreach (Camera* cam, cameras) {
		foreach (Camera* cam_other, other.cameras) {
			if (cam->get_frame_number() == cam_other->get_frame_number()) {
				common_this.push_back(cam->number);
				common_other.push_back(cam_other->number);
			}
		}
	}
}

void Reconstruction::number_items()
{
	for (size_t i=0; i<cameras.size(); i++)
		cameras[i]->number = i;
	for (size_t i=0; i<structure.size(); i++)
		structure[i]->number = i;
}

// FIXME reject tracks which were not inliers in BOTH reconstructions
void Reconstruction::merge(Reconstruction &other)
{
	std::set<Camera*> new_cams;
	std::set<Structure*> new_structure;

	std::map<Structure*,Structure*> s2s;
	std::map<Camera*,Camera*> c2c;

	foreach (Structure* pt_other, other.structure) {
		size_t track_number = pt_other->get_track_number();
		bool found = false;
		foreach (Structure* pt, structure) {
			if (pt->get_track_number() == track_number) {
				found = true;
				s2s[pt_other] = pt;
				break;
			}
		}
		if (!found) {
			new_structure.insert(pt_other);
			this->push_structure(pt_other);
		}
	}

	foreach (Camera* cam_other, other.cameras) {
		size_t frame_number = cam_other->get_frame_number();
		bool found = false;
		foreach (Camera* cam, cameras) {
			if (cam->get_frame_number() == frame_number) {
				found = true;
				c2c[cam_other] = cam;
				break;
			}
		}
		if (!found) {
			new_cams.insert(cam_other);
			this->push_camera(cam_other);
			printf("adding camera %d\n", cam_other->get_frame_number());
		}
	}

	// Now add any measurements back
	MeasurementVisitor mv(&other);
	while (mv.next()) {
		if (new_cams.count(mv.camera()) == 1 &&
				new_structure.count(mv.structure()) == 1) {
			push_measurement(mv.camera(),
					mv.structure(),
					mv.measurement());
		} else if (new_cams.count(mv.camera()) == 1) {
			push_measurement(mv.camera(),
					s2s[mv.structure()],
					mv.measurement());
		} else if (new_structure.count(mv.structure()) == 1) {
			push_measurement(c2c[mv.camera()],
					mv.structure(),
					mv.measurement());
		}
	}

	// numbering is messed after a merge
	number_items();
}


///////////////////////////////////////////////////////////////////////////////
// Non-class funcitons ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

mat normalizing_homography(mat points)
{
	int n = points.size2();

	vec xyzbar = sum2(points);

	xyzbar /= n;

	double distbar = 0;
	for (int i=0; i<n; i++) {
		double dx = points(0, i) - xyzbar[0];
		double dy = points(1, i) - xyzbar[1];
		double dz = points(2, i) - xyzbar[2];
		distbar += sqrt(dx*dx + dy*dy + dz*dz);
	}
	distbar /= n;

	double theta = sqrt(2) / distbar;

	mat H(3,3);
	H(0,0) = theta;  H(0,1) =    0.;   H(0,2) = -xyzbar[0]*theta;
	H(1,0) =    0.;  H(1,1) = theta;   H(1,2) = -xyzbar[1]*theta;
	H(2,0) =    0.;  H(2,1) =    0.;   H(2,2) =          1.;

	return H;
}

void five_point_camera_pencil(const mat &points, mat &A, mat &B)
{
	mat H = normalizing_homography(points);

	mat design(3,5);
	design = dot(H, points);

	vec v1, v2;
	nullspace(design, v1, v2);
	assert(v1.size() == 5);

	mat five_points(subrange(design, 0,3, 0,4));

	v1.resize(4);
	A = five_points*v1;

	v2.resize(4);
	B = five_points*v2;

	mat Hinv = inv(H);
	A = dot(Hinv, A);
	B = dot(Hinv, B);
}

vec calc_x6_from_design_mat(double a, double b, double c, double d, double e)
{
	/* This should match the matrix in step 6 above, equation (9) in [1] */
	mat X6null(6,4);
	X6null(0,0) = e-d;   /* first row */
	X6null(0,1) =  0 ;
	X6null(0,2) =  0 ;
	X6null(0,3) = a-b;

	X6null(1,0) = e-c;   /* second row */
	X6null(1,1) =  0 ;
	X6null(1,2) =  a ;
	X6null(1,3) =  0 ;

	X6null(2,0) = d-c;   /* third row */
	X6null(2,1) =  b ;
	X6null(2,2) =  0 ;
	X6null(2,3) =  0 ;

	X6null(3,0) =  0 ;   /* fourth row */
	X6null(3,1) =  e ;
	X6null(3,2) =  0 ;
	X6null(3,3) = a-c;

	X6null(4,0) =  0 ;   /* fifth row */
	X6null(4,1) = e-b;
	X6null(4,2) = a-d;
	X6null(4,3) =  0 ;

	X6null(5,0) =  0 ;   /* sixth row */
	X6null(5,1) =  0 ;
	X6null(5,2) =  d ;
	X6null(5,3) = b-c;

	/* Determine the position of the 6th world point X6 */
	vec Xp;
	nullspace(X6null, Xp);

	return Xp;
}

/* See paragraph after equation 16 in torr97robust for the equation which I
 * used to derive the following coefficients. */
#define accum_cubic_coeffs(x,y,z, sgn) \
p = t1[x]*t1[y]; \
q = t2[x]*t1[y] + t1[x]*t2[y]; \
d += sgn *  p*t1[z]; \
c += sgn * (q*t1[z] + p*t2[z]); \
b += sgn * (t2[x]*t2[y]*t1[z] + q*t2[z]); \
a += sgn *  t2[x]*t2[y]*t2[z];

// FIXME In probably 1/2 hour of hacking, this can be extended to N views of 6
// points!
// FIXME add tests first though.
void six_point_three_view(std::vector<ThreeFrameTrack> &tracks,
		std::vector<ProjectiveReconstruction> &out)
{
	int i,j,k;
	mat points(3,18);
	for (i=0; i<6; i++)
		for (j=0; j<3; j++)
			for (k=0; k<3; k++) {
				PointFeature *pf = tracks[i][j];
				points(k,6*j+i) = (*pf)[k];
			}

	/* See equation (7.2) p179 of HZ; this is the DLT for solving cameras. */
	/* Chose wi = 1, i.e. the homogenous component of each image
	 * location is 1. */
	mat As[3];
	mat Bs[3];
	mat ws(3,5);

	for (i=0; i<3; i++) {

		/* Extract pencil of camera matricies */
		submat fivepoints(points, range(0,3), range(6*i,6*i+5));
		five_point_camera_pencil(fivepoints, As[i], Bs[i]);

		/* Calculate Q */
		vec x6(column(points,6*i+5));
		mat x6cross = crossmat(x6);
		mat tmp = dot(transpose(As[i]), x6cross);
		mat Qa = dot(tmp, Bs[i]);
		mat Q = Qa + transpose(Qa);

		/* Read the coefficients w^i from Q and put into the ws matrix */
		ws(i,0) = Q(0,1);
		ws(i,1) = Q(0,2);
		ws(i,2) = Q(1,2);
		ws(i,3) = Q(1,3);
		ws(i,4) = Q(2,3);
	}

	/* Find a basis for the null space of ws */
	vec t1, t2;
	nullspace(ws, t1, t2);

	/* The svd gives us the basis for the nullspace of ws in which the t
	 * vector lives, such that t = beta*t1+alpha*t2. However, there is a
	 * cubic constraint on the elements of t, such that we can substitute
	 * and solve for alpha. See equation (10) in [1] */
	double a,b,c,d,p,q;
	a=b=c=d=0;
	accum_cubic_coeffs(0,1,3,  1);
	accum_cubic_coeffs(0,1,4, -1);
	accum_cubic_coeffs(0,2,4,  1);
	accum_cubic_coeffs(0,3,4, -1);
	accum_cubic_coeffs(1,2,3, -1);
	accum_cubic_coeffs(1,3,4,  1);

	/* Assume beta = 1, hope for the best FIXME */
	/* If a=b=c=0 and d!=0, then alpha=0; FIXME find beta instead */
	double a1 = b/a, b1 = c/a, c1 = d/a;
	a = a1;
	b = b1;
	c = c1;

	double alpha, alphas[3];
	int nroots = gsl_poly_solve_cubic(a,b,c, alphas+0,alphas+1,alphas+2);

	/* Check each solution for alpha */
	for (int ia=0; ia<nroots; ia++) {
		dprintf("#################### Alpha%d ###################\n",ia);
		alpha = alphas[ia];

		double e;
		a = t1[0] + alpha*t2[0];
		b = t1[1] + alpha*t2[1];
		c = t1[2] + alpha*t2[2];
		d = t1[3] + alpha*t2[3];
		e = t1[4] + alpha*t2[4];

		/* Add a new ProjectiveReconstruction */
		int n = out.size();
		out.push_back(ProjectiveReconstruction());


		/* Push the world position for the first five points */
		PointStructure *Es[5];
		for (i=0; i<4; i++) {
			Es[i] = new PointStructure(tracks[i][0]->get_track());
			(*(Es[i]))[i] = 1.0;
		}
		Es[4] = new PointStructure(tracks[4][0]->get_track());
		fill(*(Es[4]), 1.0);

		/* Find X6 from the chi vector and add to the reconstruction */
		PointStructure *Xp = new PointStructure(tracks[5][0]->get_track());
		Xp->assign(calc_x6_from_design_mat(a,b,c,d,e));

		/* Push structure into new reconstruction */
		for (j=0; j<5; j++) 
			out[n].push_structure(Es[j]);
		out[n].push_structure(Xp);

		/* Find good values of u,v for each camera */
		for (i=0; i<3; i++) {
			dprintf("  ##------------------ i=%d -------------------\n",i);

			/* Project X6 with A and B */
			/* DO NOT NORMALIZE, it breaks the next step */
			vec AX = dot(As[i], *Xp);
			vec BX = dot(Bs[i], *Xp);

			/* Find mu and nu with smallest algebraic error; see step 7*/
			mat M(3,3);
			for (j=0; j<3; j++) {
				M(j,0) = AX[j];
				M(j,1) = BX[j];
				M(j,2) = points(j,6*i+5);
			}

			vec munu;
			nullspace(M, munu);
			double mu = munu[0];
			double nu = munu[1];

			Frame* frame = tracks[0][i]->get_frame();
			ProjectiveCamera *P = new ProjectiveCamera(frame);
			P->assign(mu*As[i] + nu*Bs[i]);

			/* Store camera and visibility */
			out[n].push_camera(P);

			/* Store actual measurements */
			for (j=0; j<5; j++) 
				out[n].push_measurement(P, Es[j], tracks[j][i]);
			out[n].push_measurement(P, Xp, tracks[5][i]);

			//			for (j=0; j<5; j++) 
			//				printf("repro error %d=%g\n",j, P->reprojection_error(Es[j],tracks[j][i]));
			//			printf("repro error 5=%g\n",P->reprojection_error(Xp,tracks[5][i]));

		}
	}
}


void find_epipoles_from_fundamental(const mat &F, vec &e1, vec &e2)
{
	nullspace(F, e1);

	mat Ft;
	Ft = transpose(F);
	nullspace(Ft, e2);

	/* scale so ex^2 + ey^2 == 1 */
	double e1x = e1[0];
	double e1y = e1[1];
	double s1 = sqrt(e1x*e1x + e1y*e1y);
	e1[0] = e1x/s1;
	e1[1] = e1y/s1;
	e1[2] = e1[2]/s1;

	double e2x = e2[0];
	double e2y = e2[1];
	double s2 = sqrt(e2x*e2x + e2y*e2y);
	e2[0] = e2x/s2;
	e2[1] = e2y/s2;
	e2[2] = e2[2]/s2;
}


void find_optimal_points_given_fundamental(
		const vec &x1, const vec &x2,
		const mat &F,
		vec &x1p, vec &x2p)
{
	mat T1 = eye(3);
	T1(0,2) = -x1[0];
	T1(1,2) = -x1[1];
	mat T1inv = inv(T1);

	mat T2 = eye(3);
	T2(0,2) = -x2[0];
	T2(1,2) = -x2[1];
	mat T2inv = inv(T2);

	/*        -T         -1  */
	/* Fp = T2   * F * T1    */
	mat Fp = dot(transpose(T2inv), F);
	Fp = dot(Fp, T1inv);

	vec e1, e2;
	find_epipoles_from_fundamental(Fp, e1, e2);

	mat R1(3,3);
	double e1x = e1[0];
	double e1y = e1[1];
	R1(0,0) =  e1x; R1(0,1) =  e1y; R1(0,2) =  0.0;
	R1(1,0) = -e1y; R1(1,1) =  e1x; R1(1,2) =  0.0;
	R1(2,0) =  0.0; R1(2,1) =  0.0; R1(2,2) =  1.0;

	mat R2(3,3);
	double e2x = e2[0];
	double e2y = e2[1];
	R2(0,0) =  e2x; R2(0,1) =  e2y; R2(0,2) =  0.0;
	R2(1,0) = -e2y; R2(1,1) =  e2x; R2(1,2) =  0.0;
	R2(2,0) =  0.0; R2(2,1) =  0.0; R2(2,2) =  1.0;

	mat Fpp(3,3);
	mat tmp(3,3);
	tmp = dot(R2, Fp);
	Fpp = dot(tmp, transpose(R1));

	/* Now, Fpp must have the form:
	 *
	 * [ f*fp*d   -fp*c   -fp*d ]
	 * [ -f*b      a        b   ]
	 * [ -f*d      c        d   ]
	 *
	 * See page HZ 317 */

	double f  =  e1[2];
	double fp =  e2[2];
	double a  =  Fpp(1,1);
	double b  =  Fpp(1,2);
	double c  =  Fpp(2,1);
	double d  =  Fpp(2,2);
	dprintmat(Fpp);

	double f2  = f*f;
	double fp2 = fp*fp;
	double a2  = a*a;
	double b2  = b*b;
	double c2  = c*c;
	double d2  = d*d;

	double a3  = a2*a;
	double b3  = b2*b;
	double c3  = c2*c;
	double d3  = d2*d;

	double f4  = f2*f2;
	double fp4 = fp2*fp2;
	double a4  = a2*a2;
	double b4  = b2*b2;
	double c4  = c2*c2;
	double d4  = d2*d2;

	/* Yacas script to find coeffs of equation 12.7 in HZ (P317)
	 * In> e := ExpandBrackets(t*((a*t+b)^2+fp^2*(c*t+d)^2)^2-(a*d-b*c)*(1+f^2*t^2)^2*(a*t+b)*(c*t+d))
	 * In> Coef(e, t, 0 .. 6)
	 *
	 * Then take the Yacas output and feed it into Python with the below
	 * commands to make it into C code:
	 * >>> import re
	 * >>> x = """Paste here"""
	 * >>> re.sub(r'(f?[abcdpf])\^([234])',r'\1\2', x)
	 *
	 * Then reverse the result and paste it below (because rpoly takes the
	 * coefs in decreasing order).
	 */

	double coeffs[] = {
		a*b*c2*f4-a2*c*d*f4,
		a4+2*a2*fp2*c2-a2*d2*f4+b2*c2*f4+fp4*c4,
		4*a3*b+4*a2*fp2*c*d-2*a2*c*d*f2+4*a*b*fp2*c2+2*a*b*c2*f2-a*b*d2*f4+b2*c*d*f4+4*fp4*c3*d,
		6*a2*b2+2*a2*fp2*d2-2*a2*d2*f2+8*a*b*fp2*c*d+2*b2*fp2*c2+2*b2*c2*f2+6*fp4*c2*d2,
		4*a*b3+4*a*b*fp2*d2+a*b*c2-2*a*b*d2*f2-a2*c*d+4*b2*fp2*c*d+2*b2*c*d*f2+4*fp4*c*d3,
		b4+2*b2*fp2*d2+b2*c2+fp4*d4-a2*d2,
		b2*c*d-a*b*d2,
	};

	double zerosr[6], zerosi[6];
	int nroots = rpoly(coeffs, 6, zerosr, zerosi);
	if (nroots == -1) {
		// leading coeff is zero, try again
		nroots = rpoly(coeffs+1, 5, zerosr, zerosi);
	}
	assert(nroots != -1);
	assert(nroots != -2);

	double best_cost = 1e100;
	double best_t = 3.14159;
	double t;
	int i;
	for (i=0; i<nroots; i++) {
		t = zerosr[i];
		double ctpd2 = (c*t+d)*(c*t+d);
		double cost = t*t/(1+f2*t*t) + ctpd2/((a*t+b)*(a*t+b) + fp2*ctpd2);
		dprintf("cost_%d=%g; t_%d=%g\n",i,cost,i,t);
		if (cost < best_cost) {
			best_cost = cost;
			best_t = t;
		}
	}

	/* Check asymptotic cost, i.e. s(t_inf) */
	double cost_inf = 1/f2 + c2/(a2 + fp2*c2);
	assert (best_cost < cost_inf); /* not sure how to handle this now? */

	t = best_t;

	double l1x = t*f;
	double l1y = 1.0;
	double l1z = -t;
	double l2x = -fp*(c*t+d);
	double l2y = a*t+b;
	double l2z = c*t+d;

	x1p.resize(3);
	x1p[0] = -l1x*l1z;
	x1p[1] = -l1y*l1z;
	x1p[2] =  l1x*l1x + l1y*l1y;

	x2p.resize(3);
	x2p[0] = -l2x*l2z;
	x2p[1] = -l2y*l2z;
	x2p[2] =  l2x*l2x + l2y*l2y;

	/* Transform back */
	x1p = dot(transpose(R1), x1p);
	x1p = dot(T1inv, x1p);
	x2p /= x2p[2];

	x2p = dot(transpose(R2), x2p);
	x2p = dot(T2inv, x2p);
	x2p /= x2p[2];
}


void find_fundamental_from_camera_matricies(
		const mat &P1, const mat &P2, mat &F)
{
	vec C;
	nullspace(P1, C);
	vec e2 = dot(P2, C);
	mat e2cross = crossmat(e2);
	F = dot(P2, pinv(P1));
	F = dot(e2cross, F);
}

// FIXME totally untested! it only compiles!
void factorize_projective_reconstruction(
		const std::vector<std::vector<PointFeature*> > points, 
		const std::vector<Frame*> frames,
		ProjectiveReconstruction &pr)
{
	size_t n = points.size();
	size_t m = points[0].size();

	mat pmat(3*n,m);
	for (size_t i=0; i<n; i++)
		for (size_t j=0; j<m; j++)
			for (size_t k=0; k<3; k++)
				pmat(3*i+k,j) = (*(points[i][j]))[k];

	fmat factorize = pmat;
	mat lambda(n,m);
	fill(lambda, 1.0);

	(void) pr;
	(void) frames;

	mat Ps, Xs;
	int iterations = 10;
	for (int l=0; l<iterations; l++) {
		/* The left four columns of U hold the stacked projection
		 * matricies, and the first four rows of Vt hold the homogenous
		 * world points */
		bmat s;
		fmat U, Vt;
		s = svd(factorize, U, Vt);

		/* Do a rank-4 truncation */
		vec scale_factor(4);
		for (size_t i=0; i<4; i++)
			scale_factor[i] = s(i,i);
		Ps = subrange(U,0,3*n, 0,4);
		Ps = Ps*scale_factor;
		Xs = subrange(Vt,0,4,0,m);

		if (l == iterations)
			break;

		/* Re-calculate projective depths */
		for (size_t i=0; i<n; i++)
			for (size_t j=0; j<m; j++) {
				lambda(i,j) = 0.;
				for (size_t k=0; k<4; k++)
					lambda(i,j) += Ps(3*i+2,k)*Xs(i,k);
			}

		for (size_t i=0; i<n; i++)
			for (size_t j=0; j<m; j++)
				for (size_t k=0; k<3; k++)
					factorize(3*i+k,j) = lambda(i,j)*pmat(3*i+k,j);
	}
}

vec triangulate_dlt(const mat &P1, const mat &P2, const vec x1, const vec x2)
{
	mat design(4,4);
	for (int i=0; i<4; i++) {
		design(0,i) = x1[0]*P1(2,i) - P1(0,i);
		design(1,i) = x1[1]*P1(2,i) - P1(1,i);
		design(2,i) = x2[0]*P2(2,i) - P2(0,i);
		design(3,i) = x2[1]*P2(2,i) - P2(1,i);
	}
	vec X;
	nullspace(design, X);
	return X;
}

mat align_oneview(mat PBplusPA, vec h,
		std::vector<PointStructure*> &XAs,
		std::vector<PointStructure*> &XBs)
{
	assert(XAs.size() == XBs.size());
	size_t nmatch = XAs.size();

	/* Use simple SVD method */
	mat M(3*nmatch, 4);
	mat C(3*nmatch, 1);
	vec a(4);
	vec b(3);
	vec XA(4);
	vec XB(4);

	size_t row = 0;
	for (size_t i=0; i<nmatch; i++) {
		XA.assign(*XAs[i]);
		XB.assign(*XBs[i]);

		a = dot(PBplusPA, XA);

		for (int j=0; j<3; j++)
			b[j]  = h[j]*XB[3] - h[3]*XB[j];

		subrange(M, row,row+3, 0,4) = outer_prod(b, XA);

		for (int j=0; j<3; j++)
			C(row+j,0) = a[3]*XB[j] - a[j]*XB[3];
		row += 3;
	}
	assert(3*nmatch==row);

	// Least squares fit via pseudo inverse.
	mat v = dot(pinv(M), C);

	//          +        T
	// H(v) = Pb  Pa + hv
	//
	mat H = PBplusPA + outer_prod(h,ravel(v));

	return H;
}

double trace(mat const &m)
{
	double ret=0;
	for (size_t i=0; i<m.size1(); i++) 
		ret += m(i,i);
	return ret;
}

mat find_euclidean_upgrade_calibrated(ProjectiveReconstruction &pr, mat K, bool flip)
{
	size_t ncams = pr.cameras.size();
	mat M(3*pr.cameras.size(), 4);
	mat Kinv = inv(K);

	/* Remove the K matrix by left-multiplying by K^-1. While the algorithm
	 * given in Oliensis's paper is, in theory, for arbitrary K, the
	 * finite-precision reality is that it is not. Because the symbolic
	 * expressions are so unstable, I could not get the algorithm to work
	 * unless I pre-scaled the matricies to have K = I3x3. */
	for (size_t i=0; i<pr.cameras.size(); i++) {
		ProjectiveCamera* pc = pr.pcam(i);
		mat pcc = dot(Kinv, *pc);
		*(mat*)pc = pcc;

		/* And accumulate a vertical stack of the camera matricies */
		subrange(M,3*i,3*i+3, 0,4) = pcc;
	}

	/* Force M.T * M ~= I4x4, aka make the columns orthogonal for numeric
	 * stability. Note that this does not make M.T*M=I, but only to a scale
	 * factor; in my Python prototype, found that if I did scale by the
	 * singular values (which makes M.T*M exactly = I4x4), then the problem
	 * became ill-conditioned. I'm still not sure the reason for this. */
	bmat s;
	fmat U, Vt;
	s = svd(M, U, Vt);
	mat orthogonalize = -trans(Vt);

	/* sanity... delete me! */
	M = dot(M,orthogonalize);
	mat tmp = dot(trans(M),M);
	printf("tmp should be something like ident\n");
	dpmat(tmp);

	/* Accumulate the coefficients in the A matrix; see the Oliensis paper.  */
	mat A_(mzeros(10,10));
	mat w = eye(3);
	double w1=w(0,0);
	double w2=w(0,1);
	double w3=w(0,2);
	double w4=w(1,1);
	double w5=w(1,2);
	double w6=w(2,2);
	double trww = trace(dot(w,w));
	for (size_t i=0; i<pr.cameras.size(); i++) {
		ProjectiveCamera* pc = pr.pcam(i);
		mat pcc = dot(*pc, orthogonalize);
		//		mat pcc = *pc;
		pc->assign(pcc);
		double p0=pcc(0,0);
		double p1=pcc(0,1);
		double p2=pcc(0,2);
		double p3=pcc(0,3);
		double p4=pcc(1,0);
		double p5=pcc(1,1);
		double p6=pcc(1,2);
		double p7=pcc(1,3);
		double p8= pcc(2,0);
		double p9= pcc(2,1);
		double p10=pcc(2,2);
		double p11=pcc(2,3);

		/* DUCK! see the .mc file in scripts/ and maixma.py also in
		 * scripts/ for the code used to generate these coefficients. */
#include "euclidean_upgrade_coeffs.cpp"

		mat A__(10,10);
		fill(A__, A);
		A_ += A__;
	}
	A_ /= norm(A_);
	dpmat(A_);
	mat invsqrtB(10,10);
	/* if you think this is dumb, you are right. feel free to fix it. FIXME */
	double B[] = {
		2,0,0,0,0,0,0,0,0,0,
		0,4,0,0,0,0,0,0,0,0,
		0,0,4,0,0,0,0,0,0,0,
		0,0,0,4,0,0,0,0,0,0,
		0,0,0,0,2,0,0,0,0,0,
		0,0,0,0,0,4,0,0,0,0,
		0,0,0,0,0,0,4,0,0,0,
		0,0,0,0,0,0,0,2,0,0,
		0,0,0,0,0,0,0,0,4,0,
		0,0,0,0,0,0,0,0,0,2};
	fill(invsqrtB, (double*)B);
	for (int i=0; i<10; i++)
		invsqrtB(i,i) = 1./sqrt(invsqrtB(i,i));
	tmp = dot(A_,invsqrtB);
	mat Ahat = dot(invsqrtB, tmp);
	vec ns;
	nullspace(Ahat, ns);
	vec q = dot(invsqrtB, ns);
	double Qs[] = {
		q[0], q[1], q[2], q[3],
		q[1], q[4], q[5], q[6],
		q[2], q[5], q[7], q[8],
		q[3], q[6], q[8], q[9]
	};
	mat Q(4,4); // Absolute quadric 
	fill(Q,Qs);

	/* FIXME FIXME FIXME check if Q is PSD via eigen decomposition! if it's
	 * not we've got problems */

	/* Wrench Q to have exactly rank 3... this may not be optimal */
	s = svd(Q, U, Vt);
	s(3,3) = 0.;
	tmp = dot(s, Vt);
	Q = dot(U, tmp);

	/* Recover first three columns of T from svd of Q */
	s = svd(Q, U, Vt);
	mat T13_(4,3);
	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
			T13_(i,j) = U(i,j)*sqrt(s(j,j));

	/* Recover the scale factors for each camera and correct the scale */
	/* Take camera 0 to be the reference */
	vec ui2(ncams);
	for (size_t i=0; i<ncams; i++) {
		mat p = dot(*pr.pcam(i), Q);
		tmp = dot(p,trans(*pr.pcam(i)));
		ui2[i] = trace(dot(w,tmp))/trww;
		printf("ui2[%d] = %g\n", i, ui2[i]);
		assert(ui2[i] > 0.0);
		double ui = sqrt(ui2[i]);
		/* Prevent R from having reflections */
		mat PT13 = dot(*pr.pcam(i), T13_);
		PT13 /= ui;
		if (det3(PT13) < 0.0)
			ui = -ui;
		*pr.pcam(i) /= ui;
	}

	/* and force the first camera to be the reference image */
	tmp = dot(*pr.pcam(0),T13_);
	mat tmp2 = dot(trans(K),tmp);
	fmat ftmp = tmp2;
	s = svd(ftmp, U, Vt);
	mat O = dot(trans(Vt), trans(U));
	if (det3(O) < 0.0) 
		fprintf(stderr, "det O < 0: det(O)=%g\n",det3(O));
	vec T4;
	tmp = dot(T13_, O);

	/* Center on first camera's center */
	nullspace(*pr.pcam(0), T4);

	/* Assemble the transformation */
	mat T(4,4);
	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
			T(i,j) = tmp(i,j);
	for (size_t i=0; i<4; i++)
		if (flip) 
			T(i,3) = T4[i];
		else
			T(i,3) = -T4[i];
	// Changing the sign of the 4th column of T reflects the
	// reconstruction. Unfortunately, sometimes it is needed, other times it is
	// not. I am not sure the reason behind this.

	dpmat(T);

	/* Transform cameras */
	for (size_t i=0; i<ncams; i++) {
		ProjectiveCamera *pc = pr.pcam(i);
		mat tmp = dot(*pc, T); // tmp required because of aliasing problems with pointers

		// Force orthogonality... This may not be desired...
		mat R = subrange(tmp, 0,3,0,3);
		fmat U, Vt;
		s = svd(R, U, Vt);
		printf("ortho: %g %g %g\n", s(0,0), s(1,1), s(2,2));
		R = dot(U,Vt); // Orthogonal polar compliment
		subrange(tmp, 0,3,0,3) = R;

		// Alternately...
		//		mat rr, qq;
		//		rq(R, rr, qq);
		//		dpmat(rr);
		//		mat correction = inv(rr);
		//		tmp = dot(correction, tmp);
		//		subrange(tmp,0,3,0,3) = qq; // OR

		tmp = dot(K, tmp);
		pc->assign(tmp);
	}

	/* Transform points by T^-1 */
	mat OT = dot(orthogonalize, T);
	mat OTinv = inv(OT);
	dpmat(OTinv);
	mat Oinv = inv(orthogonalize);
	mat Tinv = inv(T);
	mat TinvOinv = dot(Tinv,Oinv);
	foreach (Structure* pts, pr.structure) {
		PointStructure* pt = (PointStructure*) pts;
		vec tmp = dot(OTinv,*pt); // THESE TEMPORARIES ARE NECESSARY! ublas misses the aliasing
		//		vec tmp = dot(TinvOinv,*pt); // THESE TEMPORARIES ARE NECESSARY! ublas misses the aliasing
		//		vec tmp = dot(Tinv,*pt); // THESE TEMPORARIES ARE NECESSARY! ublas misses the aliasing
		pt->assign(tmp);
		*pt /= (*pt)[3]; // and normalize by last element to force scale

	}
	return T;
}

////////////////////////////////////////////////////////////////////////////////
// File IO routines
////////////////////////////////////////////////////////////////////////////////

// FIXME FIXME convert to qfits, and do sanity checking (i.e. magic number!!!!)
void dump_to_file(TrackedSequence &ts, const char *fn)
{
	// Give features and whatnot a number tag
	ts.number_items();

	// 4-byte               ntracks
	// 4-byte               nframes
	// 4-byte*(nframes+1)   frame offsets
	// 8-byte*2*nfeatures   packed features
	// 4-byte*2*(ntracks+1) track offsets
	// 4-byte*2*nfeatures   track heap

	FILE* fid = fopen(fn, "wb");

	// ntracks
	size_t size = ts.tracks.size();
	fwrite(&size, sizeof(size_t), 1, fid);
	// nframes
	size = ts.size();
	fwrite(&size, sizeof(size_t), 1, fid);

	// compute frame offsets
	size_t offsets[ts.size()];
	for (size_t i=0; i<ts.size(); i++) 
		offsets[i] = 0;
	size_t total = 0;
	for (size_t i=0; i<ts.size(); i++) {
		offsets[i] = total;
		total += ts[i]->size();
	}
	// write frame offsets
	for (size_t i=0; i<ts.size(); i++) {
		size = offsets[i];
		fwrite(&size, sizeof(size_t), 1, fid);
	}
	// write trailer offset for size calculation
	size = offsets[ts.size()-1] + ts[ts.size()-1]->size();
	fwrite(&size, sizeof(size_t), 1, fid);

	// write frames (features)
	for (size_t i=0; i<ts.size(); i++) {
		for (size_t j=0; j<ts[i]->size(); j++) {
			PointFeature *pf = (PointFeature*)(*(ts[i]))[j];
			double tmp = pf->x();
			fwrite(&tmp, sizeof(double), 1, fid);
			tmp = pf->y();
			fwrite(&tmp, sizeof(double), 1, fid);
		}
	}

	// write filenames (ugh, should use JSON or someting!)
	for (size_t i=0; i<ts.size(); i++) {
		size_t tmp = ts[i]->get_filename().size();
		fwrite(&tmp, sizeof(size_t), 1, fid);
		fwrite(ts[i]->get_filename().c_str(), sizeof(char), tmp+1, fid); // include trailing null
	}

	// compute track offsets
	size_t track_offsets[ts.tracks.size()];
	for (size_t i=0; i<ts.tracks.size(); i++) 
		track_offsets[i] = 0;
	total = 0;
	for (size_t i=0; i<ts.tracks.size(); i++) {
		track_offsets[i] = total;
		total += ts.tracks[i]->size();
	}
	// write track offsets
	for (size_t i=0; i<ts.tracks.size(); i++) {
		size = track_offsets[i];
		fwrite(&size, sizeof(size_t), 1, fid);
	}
	// write trailer offset for size calculation
	size = track_offsets[ts.tracks.size()-1] +
		ts.tracks[ts.tracks.size()-1]->size();
	fwrite(&size, sizeof(size_t), 1, fid);

	// write tracks (features)
	foreach (Track* tr, ts.tracks) {
		foreach (Feature *ft, *tr) {
			size_t tmp = ft->get_frame_number();
			assert (tmp < 5000);
			fwrite(&tmp, sizeof(size_t), 1, fid);
			tmp = ft->get_number();
			assert (tmp < 1000);
			fwrite(&tmp, sizeof(size_t), 1, fid);
		}
	}

	fclose(fid);
}

TrackedSequence* read_from_file(const char *fn) {
	FILE* fid = fopen(fn, "rb");
	if (fid == NULL)
		fprintf(stderr, "error opening file...");
	assert(fid);
	TrackedSequence* ts = read_from_file(fid);
	fclose(fid);
	return ts;
}


TrackedSequence* read_from_file(FILE* fid)
{
	TrackedSequence *ts = new TrackedSequence();

	// ntracks
	size_t ntracks;
	fread(&ntracks, sizeof(size_t), 1, fid);
	ts->tracks.resize(ntracks);
	// nframes
	size_t nframes;
	fread(&nframes, sizeof(size_t), 1, fid);
	ts->resize(nframes);
	// frame offsets
	size_t offsets[nframes+1];
	fread(offsets, sizeof(size_t), nframes+1, fid);
	long base;
	base = ftell(fid);
	// read frame offsets and features
	for (size_t i=0; i<nframes; i++) {
		fseek(fid, base+2*sizeof(double)*(offsets[i]), SEEK_SET);
		assert(offsets[i+1] > offsets[i]);
		size_t nfeats = offsets[i+1] - offsets[i];
		Frame* fr = (*ts)[i] = new Frame();
		fr->resize(nfeats);
		fr->set_frame_number(i);
		for (size_t j=0; j<nfeats; j++) {
			PointFeature *pf = new PointFeature();
			double tmp[2];
			fread(&tmp, sizeof(double), 2, fid);
			pf->set(tmp[0], tmp[1]);
			assert(tmp[0] > 1.);
			assert(tmp[0] < 1000.);
			assert(tmp[1] > 1.);
			assert(tmp[1] < 1000.);
			(*fr)[j] = pf;
			pf->set_frame(fr);
			pf->set_frame_number(i);
			pf->set_number(j);
		}
	}

	// read filenames (ugh, should use JSON or someting!)
	for (size_t i=0; i<ts->size(); i++) {
		size_t tmp;
		fread(&tmp, sizeof(size_t), 1, fid);
		char buf[tmp+1];
		fread(buf, sizeof(char), tmp+1, fid);
		ts->at(i)->set_filename(buf);
	}

	// read track offsets
	size_t track_offsets[ntracks+1];
	fread(track_offsets, sizeof(size_t), ntracks+1, fid);
	base = ftell(fid);
	// read tracks (frame nr# and feature nr#)
	for (size_t i=0; i<ntracks; i++) {
		fseek(fid, base+2*sizeof(size_t)*(track_offsets[i]), SEEK_SET);
		assert(track_offsets[i+1] > track_offsets[i]);
		size_t npts = track_offsets[i+1] - track_offsets[i];
		Track* tr = ts->tracks[i] = new Track();
		tr->resize(npts);
		tr->set_number(i);
		assert(tr->get_number()==i);
		for (size_t j=0; j<npts; j++) {
			size_t tmp;
			fread(&tmp, sizeof(size_t), 1, fid);
			Frame* fr = (*ts)[tmp];
			fread(&tmp, sizeof(size_t), 1, fid);
			Feature* ft = (*fr)[tmp];
			(*tr)[j] = ft;
			ft->set_track(tr);
		}
	}
	return ts;
}

void dump_reconstruction_to_file(ProjectiveReconstruction& pr, const char *fn) 
{
	FILE* fid = fopen(fn, "wb");
	assert(fid);
	dump_reconstruction_to_file(pr, fid);
	fclose(fid);
}

void dump_reconstruction_to_file(ProjectiveReconstruction& pr, FILE* fid) 
{
	// 4-byte                 ncams
	// 4-byte                 npoints
	// 4-byte                 nmeasurements
	// 8-byte*12*ncams        cameras
	// 8-byte*4*npts          points
	// cam#/str#/frame#/feat# measurements

	// need consistent numbering
	pr.number_items();

	// File header (ncams,npoints,nmeasurements)
	size_t ncams = pr.cameras.size();
	fwrite(&ncams, sizeof(size_t), 1, fid);
	size_t npoints = pr.structure.size();
	fwrite(&npoints, sizeof(size_t), 1, fid);
	size_t nmeasurements = pr.measurements.size();
	fwrite(&nmeasurements, sizeof(size_t), 1, fid);

	// Cameras
	for (size_t i=0; i<ncams; i++) {
		ProjectiveCamera* pc = pr.pcam(i);
		// FIXME there is a way to get at the raw matrix data...
		// somehow...
		size_t frame_number = pc->get_frame_number();
		fwrite(&frame_number, sizeof(size_t), 1, fid);
		double data[3][4];
		for (int j=0; j<3; j++) 
			for (int k=0; k<4; k++) 
				data[j][k] = (*pc)(j,k);
		fwrite(&data, sizeof(double), 12, fid);
	}

	// Structure
	for (size_t i=0; i<npoints; i++) {
		PointStructure* ps = (PointStructure*)pr.structure[i];
		// FIXME there is a way to get at the raw matrix data...
		// somehow...
		double data[4];
		for (int k=0; k<4; k++) 
			data[k] = (*ps)(k);
		fwrite(&data, sizeof(double), 4, fid);
	}

	// Measurements
	MeasurementVisitor mv(&pr);
	while (mv.next()) {

		// Number of the camera
		ProjectiveCamera* cam = (ProjectiveCamera*) mv.camera();
		size_t tmp = cam->number;
		fwrite(&tmp, sizeof(int), 1, fid);

		// Number of the point
		// ugly, need to hide this structure numbering crap
		PointStructure* ss = (PointStructure*) mv.structure();
		tmp = ss->number;
		fwrite(&tmp, sizeof(int), 1, fid);

		// Frame number and feature number in that frame
		tmp = mv.measurement()->get_frame_number();
		fwrite(&tmp, sizeof(int), 1, fid);
		tmp = mv.measurement()->get_number();
		fwrite(&tmp, sizeof(int), 1, fid);
	}
}

ProjectiveReconstruction *read_reconstruction_from_file(TrackedSequence& ts, const char *fn) 
{
	FILE* fid = fopen(fn, "rb");
	assert(fid);
	ProjectiveReconstruction *pr = read_reconstruction_from_file(ts, fid);
	fclose(fid);
	return pr;
}

ProjectiveReconstruction *read_reconstruction_from_file(TrackedSequence& ts, FILE* fid)
{
	ProjectiveReconstruction *pr = new ProjectiveReconstruction;

	// File header (ncams,npoints,nmeasurements)
	size_t ncams;
	fread(&ncams, sizeof(size_t), 1, fid);
	size_t npoints;
	fread(&npoints, sizeof(size_t), 1, fid);
	size_t nmeasurements;
	fread(&nmeasurements, sizeof(size_t), 1, fid);

	pr->cameras.resize(ncams);
	pr->structure.resize(npoints);

	// Cameras
	for (size_t i=0; i<ncams; i++) {
		ProjectiveCamera* pc = new ProjectiveCamera;
		// FIXME there is a way to get at the raw matrix data...
		// somehow...
		size_t frame_number;
		fread(&frame_number, sizeof(size_t), 1, fid);
		assert(frame_number < ts.size());
		double data[3][4];
		fread(&data, sizeof(double), 12, fid);
		for (int j=0; j<3; j++) 
			for (int k=0; k<4; k++) 
				(*pc)(j,k) = data[j][k];
		pc->set_number(i);
		pc->set_frame(ts[frame_number]);
		pr->cameras[i] = pc;
	}

	// Structure
	for (size_t i=0; i<npoints; i++) {
		PointStructure* ps = new PointStructure;
		// FIXME there is a way to get at the raw matrix data...
		// somehow...
		double data[4];
		fread(&data, sizeof(double), 4, fid);
		for (int k=0; k<4; k++) 
			(*ps)(k) = data[k];
		ps->set_number(i);
		pr->structure[i] = ps;
	}

	// Measurements
	for (size_t i=0; i<nmeasurements; i++) {
		size_t cam_number;

		fread(&cam_number, sizeof(size_t), 1, fid);

		size_t point_number;
		fread(&point_number, sizeof(size_t), 1, fid);

		size_t frame_number;
		fread(&frame_number, sizeof(size_t), 1, fid);

		size_t feature_number;
		fread(&feature_number, sizeof(size_t), 1, fid);

		assert(cam_number < ncams);
		assert(point_number < npoints);
		assert(frame_number < ts.size());
		assert(feature_number < ts[frame_number]->size());
		Frame* fr = ts[frame_number];
		pr->push_measurement(pr->cameras[cam_number],
				pr->structure[point_number],
				(*fr)[feature_number]);
		pr->structure[point_number]->set_track(
				(*fr)[feature_number]->get_track());
	}

	return pr;
}

void reconstruct_sequential_subsets(TrackedSequence &ts, size_t ransac_rounds, int last_frame,
		std::vector<Reconstruction*> *subsets)
{

	size_t end;
	if (last_frame==-1)
		end = ts.size();
	else
		end = last_frame + 1;

	assert(end <= ts.size());

	for (size_t i=0; i+2<end; i+=2) {
		printf("i=%d\n",i);
		ThreeViewSixPointRANSACDriver drv(ts, i);
		if (drv.search(ransac_rounds)) { // forget about failing reconstructions
			ProjectiveReconstruction *pr = new ProjectiveReconstruction;
			drv.best_fit.copy_into(pr);
			pr->estimate_errors();
			subsets->push_back(pr);
		} else { 
			printf("Failed to make subset for i=%d\n", i);
		}
	}
}

void reconstruct_keyframes(
		TrackedSequence &ts,
		size_t ransac_rounds,
		const std::vector<size_t> &keyframes,
		std::vector<Reconstruction*> *subsets,
		Config *config)
{
//	for (size_t i=0; i+1 < keyframes.size(); i++) {
	for (size_t i=0; i+2 < keyframes.size(); i+=2) {
		printf("i=%d\n",i);
		ThreeViewSixPointRANSACDriver drv(ts, 
			keyframes[i],
//			(keyframes[i]+keyframes[i+1]) / 2,
			keyframes[i+1],
			keyframes[i+2]);
		if (drv.search(ransac_rounds)) { // forget about failing reconstructions
			ProjectiveReconstruction *pr = new ProjectiveReconstruction;
			drv.best_fit.copy_into(pr);
			pr->estimate_errors();
			pr->set_tracked_sequence(&ts);
			subsets->push_back(pr);
			pr->set_config(*config);
		} else { 
			printf("Failed to make subset for i=%d\n", i);
		}
	}
}

// FIXME update this
void reconstruct_keyframes_nview(
		TrackedSequence &ts,
		size_t ransac_rounds,
		const std::vector<size_t> &keyframes,
		std::vector<Reconstruction*> *subsets,
		Config *config)
{
	for (size_t i=0; i+1<keyframes.size(); i++) {
		printf("i=%d\n",i);
		double error_thresh;
		ProjectiveReconstruction *pr = robust_sixpoint_n_view_tracks(
				ts, keyframes[i], keyframes[i+1],
				ransac_rounds,
				&error_thresh);
		subsets->push_back(pr);
	}
}

void bundle_adjust_subsets(const
		std::vector<Reconstruction*> &subsets)
{
	for (size_t i=0; i<subsets.size(); i++) {
		subsets[i]->bundle_adjust();
		subsets[i]->trim_outliers();
	}
}

ProjectiveReconstruction* reconstruct_hierarchical(
		TrackedSequence &ts,
		size_t ransac_rounds,
		int last_frame)
{
	std::vector<Reconstruction*> prs;
	printf("Reconstructing subsets\n");
	printf("======================\n");
	reconstruct_sequential_subsets(ts, ransac_rounds, last_frame, &prs);
	Config config;
	return merge_hierarchical(ts, prs);
}

// the type unsaftey of this function is unfortunate.
static ProjectiveReconstruction *pop_queue(
		std::vector<Reconstruction*> &queue,
		size_t i) {
	ProjectiveReconstruction *ret = 
		(ProjectiveReconstruction*)(queue[0]);
	queue.erase(queue.begin()+i, queue.begin()+i+1);
	return ret;
}

ProjectiveReconstruction* merge_hierarchical(
		TrackedSequence &ts,
		std::vector<Reconstruction*> &queue)
{
	assert(ts.size() >= 3);
//	assert((ts.size()-3)&1==0); // think about it...

	printf("Hierarchically merging\n");
	printf("======================\n");
	while (queue.size() != 1) {

		// Pop the top reconstructions off the stack
//		ProjectiveReconstruction *pr1 = queue[0];
//		queue.erase(queue.begin(), queue.begin()+1);
		ProjectiveReconstruction *pr1 = pop_queue(queue, 0);


		// Find the reconstruction nearest to the top which pr1 can
		// actually merge with-- it is likely that there are no
		// overlapping frames or more than one overlapping frame.
		ProjectiveReconstruction *pr2 = NULL;
		size_t i;
		for (i=0; i<queue.size(); i++) {
			std::vector<size_t> common_this, common_other;
			pr2 = (ProjectiveReconstruction*)queue[i];
			pr1->common_cameras(*pr2, common_this, common_other);
			if (common_this.size() == 1)
				// we should always exit via this break, and
				// furthermore, because of the way the queue is
				// initialized, the loop should never have to
				// go further than a couple down (i.e.
				// shouldn't have to rescan entire queue every
				// time) FIXME add check to make sure this is
				// not the case
				break; 
		}

		if (i == queue.size()) {
			// Ok, there are no common cameras between the two reconstructions.
			fprintf(stderr, "Err, hit a section where there is no common camera between two reconstructions\n");
			Json* js = reconstructions_to_json(queue);
			json::ToFile(*js, "queue_missing_common.prs");
//			return;
		}
		assert (i != queue.size()); // this should never happen

		queue.erase(queue.begin()+i,queue.begin()+i+1);
		
		// Swap reconstructions to so pr1 has the best RMS error
		// FIXME what about sub-reconstructions with small RMS but
		// poorly conditioned covariances? i.e. only very small 'angle'
		// so the 3d point is poorly estimated? in that case maybe the
		// other is better... this will have to wait until we have
		// proper covariance estimation. 
//		if (pr1->sigma2 < pr2->sigma2) {
		// CHANGE: Use reconstruction which has more cameras as a base.
		if ((pr1->cameras.size() < pr2->cameras.size()) ||
			((pr1->cameras.size() == pr2->cameras.size())
			 && (pr1->sigma2 > pr2->sigma2))	) {
			ProjectiveReconstruction *tmp;
			tmp = pr1;
			pr1 = pr2;
			pr2 = tmp;
		}

		force_positive_depth(pr1);
		force_positive_depth(pr2);

		printf("ALIGNING PR2:");
		for (size_t jj=0; jj<pr2->cameras.size(); jj++) {
			printf(" %d", pr2->pcam(jj)->get_frame_number());
		}
		printf("\n");
		printf("INTO     PR1:");
		for (size_t jj=0; jj<pr1->cameras.size(); jj++) {
			printf(" %d", pr1->pcam(jj)->get_frame_number());
		}
		printf("\n");
		printf("         PR2 RMS=%g\n", pr2->rms_reprojection_error());
		printf("         PR1 RMS=%g\n", pr1->rms_reprojection_error());
		printf("\n");
		printf("         PR2 NPTS=%d\n", pr2->structure.size());
		printf("         PR1 NPTS=%d\n", pr1->structure.size());

		// Since pr1 has less error, transform pr2 
		// KNOBS!
		if (!pr2->robust_oneview_align(*pr1)) {
			// we failed on this alignment; push them to the end of the queue
			// and hope for the best
			// 
			// FIXME detect when we're infinitely looping on two
			// reconstructions
			printf(" FAILED; trying in reverse order (swapped pr1 and pr2)\n");
			ProjectiveReconstruction *tmp;
			tmp = pr1;
			pr1 = pr2;
			pr2 = tmp;

			// Since we failed one way, try the other...
			if (!pr2->robust_oneview_align(*pr1)) {
				printf(" FAILED AT ALIGNMENT both ways; dumping to 'tmp.prs'\n");
				queue.push_back(pr1);
				queue.push_back(pr2);
				Json* js = reconstructions_to_json(queue);
				json::ToFile(*js, "queue.prs");
				continue;
			}
		}

		// Merge pr2 into pr1
		pr1->merge(*pr2);

		// Drop any nasty outliers that rose up because of the merge
		// BINARY KNOB
//		pr1->bundle_adjust(); // HACK!!
		// BINARY KNOB
		pr1->trim_outliers();

		// Do bundle adjustment of combination.
		// FIXME do shortcut which only optimizes the new
		// cameras/points, as the old ones should already be bundled.
		// This bundle adjustment adds an O(n^2) factor which is highly
		// undesireable for larger reconstructions
		double rms_error = pr1->rms_reprojection_error();
		printf("BEFORE BUNDLE rms error=%g i=%d\n",rms_error, i);
		printf("BEFORE BUNDLE mean reprojection error=%g i=%d\n",pr1->mean_reprojection_error(), i);

		// Copy reconstruction prematurely
		ProjectiveReconstruction* pr1_copy = (ProjectiveReconstruction*) pr1->copy();
		printf("RMS copy %g\n", pr1_copy->rms_reprojection_error());
		assert(fabs(rms_error-pr1_copy->rms_reprojection_error())<1e9);

//		std::set<Structure*> fake_strs;
//		printf("Faking DROP\n");
//		pr1->drop_structure(fake_strs);

		if (pr1->rms_reprojection_error() > 2) {
			printf("WARNING: RMS REPROJECTION ERROR IS BIG!\n");
			printf("WARNING: RMS REPROJECTION ERROR IS BIG!\n");
			printf("WARNING: RMS REPROJECTION ERROR IS BIG!\n");
			printf("WARNING:              %g              !\n", pr1->rms_reprojection_error());
			printf("WARNING: RMS REPROJECTION ERROR IS BIG!\n");
			printf("WARNING: RMS REPROJECTION ERROR IS BIG!\n");
			printf("WARNING: RMS REPROJECTION ERROR IS BIG!\n");
		}
		pr1->bundle_adjust();
		double rms_error_after = pr1->rms_reprojection_error();
		if (rms_error_after > rms_error) {
			printf("WARNING: RMS REPROJECTION ERROR GOT WORSE!!!!!\n");
			printf("WARNING: RMS REPROJECTION ERROR GOT WORSE!!!!!\n");
			printf("WARNING: RMS REPROJECTION ERROR GOT WORSE!!!!!\n");
			printf("WARNING:           %g -> %g                \n", rms_error, rms_error_after);
			printf("WARNING: RMS REPROJECTION ERROR GOT WORSE!!!!!\n");
			printf("WARNING: RMS REPROJECTION ERROR GOT WORSE!!!!!\n");
			printf("WARNING: RMS REPROJECTION ERROR GOT WORSE!!!!!\n");

			dump_reconstruction_to_file(*pr1_copy, "broken.pr");
		}
		printf("AFTER BUNDLE rms error=%g i=%d\n",pr1->rms_reprojection_error(), i);
		printf("AFTER BUNDLE mean reprojection error=%g i=%d\n",pr1->mean_reprojection_error(), i);
//		pr1->print_residuals();

		queue.push_back(pr1);

		delete pr2;
	}

	assert(queue.size() == 1);
	queue[0]->number_items();
	return (ProjectiveReconstruction*) queue[0];
}

void force_orthogonal_rotation(ProjectiveReconstruction *pr)
{
	for (size_t i=0; i<pr->cameras.size(); i++) {
		mat R = subrange(*pr->pcam(i), 0,3,0,3);
		bmat s;
		fmat U, Vt;
		s = svd(R, U, Vt);
		// FIXME add proper error handling? For now just bail if the
		// matrix is way off a rotation.
		assert(s(0,0) > 0.7);
		assert(s(1,1) > 0.7);
		assert(s(2,2) > 0.7);
		R = dot(U,Vt);
		subrange(*pr->pcam(i), 0,3,0,3) = R;
	}
}

// Given a projective reconstruction which is already metric, convert it to a
// euclidean reconstruction.
EuclideanReconstruction *projective_to_euclidean(ProjectiveReconstruction* pr)
{
	EuclideanReconstruction *nr = new EuclideanReconstruction;

	std::map<PointStructure*,EuclideanPointStructure*> s2s;
	foreach (Structure* st, pr->structure) {
		EuclideanPointStructure* new_st = new EuclideanPointStructure;
		PointStructure *ps = (PointStructure*)st;
		assert(new_st);
		new_st->X[0] = (*ps)[0];
		new_st->X[1] = (*ps)[1];
		new_st->X[2] = (*ps)[2];
		new_st->set_track(st->get_track());
		assert(fabs((*ps)[3] -1) < 0.001);
		s2s[ps] = new_st;
		nr->push_structure(new_st);
	}

	std::map<ProjectiveCamera*,EuclideanCamera*> c2c;
	foreach (Camera* cam, pr->cameras) {
		EuclideanCamera* new_cam = new EuclideanCamera;
		ProjectiveCamera* pc = (ProjectiveCamera*) cam;
		new_cam->set_frame(cam->get_frame());

		mat R = subrange(*pc, 0,3,0,3);
		mat rr, qq;
		rq(R, rr, qq);
		mat correction = inv(rr);
		mat tmp = dot(correction, *pc);
		new_cam->K = rr;
//		dpmat(rr);
//		dpmat(qq);
		new_cam->R = qq;
		mat tmp2 = subrange(tmp, 0,3,3,4);
		vec t = ravel(tmp2);
		new_cam->t = t;

		c2c[pc] = new_cam;
		nr->push_camera(new_cam);
	}

	MeasurementVisitor mv(pr);
	while (mv.next()) {
		assert(c2c.count((ProjectiveCamera*)mv.camera()) == 1);
		assert(s2s.count((PointStructure*)mv.structure()) == 1);
		nr->push_measurement(
				(Camera*)   c2c[(ProjectiveCamera*)mv.camera()],
				(Structure*)s2s[(PointStructure*)mv.structure()],
				mv.measurement());
	}

	assert(pr->structure.size() == nr->structure.size());
	assert(pr->cameras.size() == nr->cameras.size());
	assert(pr->cam_sees.size() == nr->cam_sees.size());
	assert(pr->visible_in.size() == nr->visible_in.size());

	nr->set_tracked_sequence(pr->get_tracked_sequence());

	return nr;
}

// really this is part of ProjectiveReconstruction
void sba_proj_euclidean(int jj, int ii, double *aj, double *bi, double *xij, void *adata)
{
	/* Don't generate compile warnings */
	(void) ii;  // bi is point
	(void) jj;  // aj is camera
	(void) adata;

	EuclideanReconstruction *er = (EuclideanReconstruction*)adata;

	// Convert bi into 3-vector P3 point
	vec X(3);
	fill(X, bi);

	// Adjust existing rotation matrix
	EuclideanCamera* cam = er->ecam(jj);
	mat R = cam->R;
	apply_euler_rotation(R, aj[0], aj[1], aj[2]);

	// Pull out translation
	vec t(3);
	fill(t, aj+3);

	vec tmp = dot(R, X) + t;
	vec est = dot(cam->K, tmp);
	est /= est[2];
	xij[0] = est[0];
	xij[1] = est[1];

	PointFeature* pf = (PointFeature*)er->get_feat(jj,ii);

	// Use a cost that reflects our uncertainty; i.e. errors on points we are
	// not confident about are not heavily penalized.
	// FIXME this is duped... eliminate duplication!
	if (er->use_mahalanobis_distance_) {
		// Note that we need to matrix-multiply by the inverse square root of
		// covariance, because we don't explicitly compute the squared error; we
		// only compute the components.
		// ... compute xij[0,1] = dot(sqrtm(inv(covariance)), xij[0,1])
		vec xx(2);
		xx(0) = xij[0] - pf->x();
		xx(1) = xij[1] - pf->y();
		vec yy = dot(pf->sigma_inv_sqrt, xx);
		xij[0] = yy(0) + pf->x();
		xij[1] = yy(1) + pf->y();
	}

	// Now make it a 'robust' error to not penalize outliers too much
	if (er->use_huber_cost_) {
		xij[0] = huber_shrink(pf->x(), xij[0], er->huber_threshold_);
		xij[1] = huber_shrink(pf->y(), xij[1], er->huber_threshold_);
	}
}


void EuclideanReconstruction::bundle_adjust()
{
	int n=structure.size();  // number of 3d pts
	int m=cameras.size();    // number of cameras
	int mcon=0; // number of cameras to keep fixed
	int cnp=6;  // number of params per cam 3 rot, 3 pos
	int pnp=3;  // number of params per point 
	int mnp=2;  // number of params per image measurement (how can this not be 2)

	size_t visible_2d_points = 0;
	foreach (Camera* cam, cameras)
		visible_2d_points += cam_sees.count(cam);

	assert(cam_sees.size() == visible_2d_points);

	number_items();

	char* vmask = new char[n*m];
	for (int i=0; i<n*m; i++)
		vmask[i] = 0;

	double *xij = new double[visible_2d_points*2];
	size_t ptij = 0;

	std::vector<CToS> pairs;
	MeasurementVisitor mv(this);
	while (mv.next()) {
		pairs.push_back(std::make_pair(mv.camera(), mv.structure()));
	}
	std::sort(pairs.begin(), pairs.end(), compare_ctos);

	PARAMETER(bool, euclidean_bundle_use_mahalanobis_distance, false);
	use_mahalanobis_distance_ = euclidean_bundle_use_mahalanobis_distance;

	foreach(CToS ctos, pairs) {
		Structure* ss = ctos.second;
		Camera* cam = ctos.first;
		PointFeature* f = (PointFeature*) measurements[ctos];
		if (use_mahalanobis_distance_) {
			f->sigma_inv_sqrt = f->sigma;
			inv2inplace(&f->sigma_inv_sqrt);
			sqrtm(f->sigma_inv_sqrt, &f->sigma_inv_sqrt);
		}
		xij[ptij++] = f->x();
		xij[ptij++] = f->y();
		vmask[ss->number*m + cam->number] = 1;
	}

	/* Pack the cameras and structure */
	int i = 0;
	double *p = new double[m*cnp + n*pnp];
	foreach (Camera* cam, cameras) {
		EuclideanCamera* ec = (EuclideanCamera*) cam;
		// first three are rotation; incremental, so start at 0
		for (int j=0; j<3; j++)
			p[i++] = 0.;
		// then translation
		for (int j=0; j<3; j++)
			p[i++] = ec->t[j];
	}
	foreach (Structure* s, structure) {
		EuclideanPointStructure* ps = (EuclideanPointStructure*) s;
		for (int j=0; j<3; j++)
			p[i++] = ps->X[j];
	}

	double info[10];
	double opts[4];
	PARAMETER(double, euclidean_bundle_init_mu, 1e-3);
	PARAMETER(double, euclidean_bundle_termination_threshold, 1e-8);
	opts[0]=euclidean_bundle_init_mu;
	opts[1]=euclidean_bundle_termination_threshold;
	opts[2]=euclidean_bundle_termination_threshold;
	opts[3]=euclidean_bundle_termination_threshold;

	PARAMETER(bool, euclidean_bundle_use_huber_cost, true);
	use_huber_cost_ = euclidean_bundle_use_huber_cost;
	if (euclidean_bundle_use_huber_cost) {
		PARAMETER(double, euclidean_bundle_huber_threshold, 30.0);
		huber_threshold_ = euclidean_bundle_huber_threshold;
	}

	PARAMETER(int, euclidean_bundle_max_iterations, 30);
	int ret = sba_motstr_levmar(n, m, mcon, vmask, p, cnp, pnp, xij, mnp,
			sba_proj_euclidean,
			NULL,     /* numeric jacobian */
//			sba_projjac_projective, /* analytic jacobian!!! */
			this,     /* extra parameter */
			euclidean_bundle_max_iterations, /* max iter */
			// KNOB
			8,        /* be verbose*/ 
			opts,     /* options */
			info);

	// FIXME be verbose here; the docs tell what each number means.
	printf("INFO:\n");
	for (i=0; i<10; i++)
		printf("%d - %g\n", i, info[i]);
	printf("RET: %d\n",ret);

	/* Unpack the cameras and structure */
	i = 0;
	foreach (Camera* cam, cameras) {
		EuclideanCamera* ec = (EuclideanCamera*) cam;
		apply_euler_rotation(ec->R, p[i], p[i+1], p[i+2]);
		fill(ec->t, p+i+3);
		i += 6;
	}
	foreach (Structure* s, structure) {
		EuclideanPointStructure *es = (EuclideanPointStructure*)s;
		fill(es->X, p+i);
		i += 3;
	}
}


// See http://en.wikipedia.org/wiki/Rotation_representation_(mathematics)
// for conventions used
void apply_euler_rotation(mat &R, double phi, double theta, double psi)
{
	double c, s;
	c = cos(phi);
	s = sin(phi);
	double _Ax[] = {1,0,0, 0,c,-s, 0, s,c};
	mat Ax(3,3);
	fill(Ax, _Ax);
	R = dot(R, Ax); 

	c = cos(theta);
	s = sin(theta);
	double _Ay[] = {c,0,s, 0,1,0, -s,0,c};
	mat Ay(3,3);
	fill(Ay, _Ay);
	R = dot(R, Ay); 

	c = cos(psi);
	s = sin(psi);
	double _Az[] = {c,-s,0, s,c,0, 0,0,1};
	mat Az(3,3);
	fill(Az, _Az);
	R = dot(R, Az); 
}

void EuclideanReconstruction::print_cameras()
{
	printf("Ps = []\n");
	for (size_t i=0; i<cameras.size(); i++) {
		printf("Ps.append(");
		EuclideanCamera* ec = ecam(i);
		mat tmp(3,4);
		subrange(tmp,0,3,0,3) = ec->R;
		tmp(0,3) = ec->t[0];
		tmp(1,3) = ec->t[1];
		tmp(2,3) = ec->t[2];
		// note that we are dropping the K matrix
		// so this is purely P = [R|t]
		pnumpy(tmp);
		printf(")\n");
	}
}

void EuclideanReconstruction::print_structure()
{
	printf("Ss = []\n");
	for (size_t i=0; i<structure.size(); i++) {
		printf("Ss.append(");
		EuclideanPointStructure *ep = (EuclideanPointStructure*)structure[i];
		pnumpy(ep->X);
		printf(")\n");
	}
}

// order by structure number, then cam number
// as required by SBA
bool less_than_pairs(const std::pair<int,int> &a, const std::pair<int,int> &b) 
{
	if (a.first < b.first)
		return true;
	return false;
}
void EuclideanReconstruction::write_blender(const char* fn)
{
	FILE* fid = fopen(fn, "wb");
	assert(fid);

	fprintf(fid, "# libmv blender camera track export; do not edit\n");
	fprintf(fid, "import Blender\n");
	fprintf(fid, "from Blender import Camera, Object, Scene, NMesh\n");
	fprintf(fid, "from Blender import Mathutils\n");
	fprintf(fid, "from Blender.Mathutils import *\n");
	fprintf(fid, "cur = Scene.GetCurrent()\n");

	// Calculate frame ordering. This exists because the camera numbers do not
	// necessarily correlate with the frame ordering.
	std::vector<std::pair<int,int> > ordering;
	ordering.reserve(cameras.size());
	for (size_t i=0; i<cameras.size(); i++) {
		EuclideanCamera* ec = ecam(i);
		int num = ec->get_frame()->get_orig_ind();
		ordering.push_back(std::make_pair(num,i));
	}
	std::sort(ordering.begin(), ordering.end(), less_than_pairs);

	fprintf(fid, "# Cameras\n");
	for (size_t i=0; i<cameras.size(); i++) {

		int camnum = ordering[i].second;
		EuclideanCamera* ec = ecam(camnum);
		int num = ec->get_frame()->get_orig_ind();
		assert(num == ordering[i].first);

		// Set up the camera
		fprintf(fid, "c%04d = Camera.New('persp')\n", num);
		// FIXME we may want to take K(0,0)/K(1,1) and push that into the
		// render aspect ratio settings.
		double f = ec->K(0,0);

		if (f < 0) // AAAH Autocalibration is flakey
			f *= -1.;

		double width = ts->get_width(); // FIXME this fails if w/h < 1.0
		// WARNING: MAGIC CONSTANT from blender source
		// Look for 32.0 in 'source/blender/render/intern/source/initrender.c'
		double lens = f / width * 32.0; 
		fprintf(fid, "c%04d.lens = %g\n", num, lens);
		fprintf(fid, "c%04d.setDrawSize(0.05)\n", num);
		fprintf(fid, "o%04d = Object.New('Camera')\n", num);
		fprintf(fid, "o%04d.name = 'libmv_cam%04d'\n", num, num);

		// Camera world matrix, which is the inverse transpose of the typical
		// 'projection' matrix as generally thought of by vision researchers.
		// Usually it is x = K[R|t]X; but here it is the inverse of R|t stacked
		// on [0,0,0,1], but then in fortran-order. So that is where the
		// weirdness comes from.
		fprintf(fid, "o%04d.setMatrix(Mathutils.Matrix(",num);
		for (int j=0; j<3; j++) {
			fprintf(fid, "[");
			for (int k=0; k<3; k++) {
				// Opengl's camera faces down the NEGATIVE z axis so we have to
				// do a 180 degree X axis rotation. The math works out so that
				// the following conditional nicely implements that.
				if (j == 2 || j == 1)
					fprintf(fid, "-");
				fprintf(fid, "%g,", ec->R(j,k)); // transposed!
			}
			fprintf(fid, "0.0],");
		}
		vec tmp = -dot(trans(ec->R), ec->t);
		fprintf(fid, "[");
		for (int j=0; j<3; j++)
			fprintf(fid, "%g,", tmp[j]);
		fprintf(fid, "1.0]))\n");

		// K matrix; not used by blender
		fprintf(fid, "k%04d = [", num);
		for (int j=0; j<3; j++) {
			fprintf(fid, "[");
			for (int k=0; k<3; k++) {
				fprintf(fid, "%g,", ec->K(j,k)); 
			}
			fprintf(fid, "],");
		}
		fprintf(fid, "]\n");

		// Link the scene and the camera together
		fprintf(fid, "o%04d.link(c%04d)\n\n", num,num);
		fprintf(fid, "cur.link(o%04d)\n\n", num);
	}

	// IPO curves
	fprintf(fid, "# Animate the camera with IpoCurves\n");
	fprintf(fid, "crender = Camera.New('persp')\n");
	fprintf(fid, "crender.lens = 35.0 # this lens value doesn't matter\n");
	fprintf(fid, "crender.setDrawSize(0.05)\n");
	fprintf(fid, "orender = Object.New('Camera')\n");
	fprintf(fid, "orender.name = \"libmv_render_cam\"\n");
	fprintf(fid, "orender.link(crender)\n");
	fprintf(fid, "cur.link(orender)\n");
	fprintf(fid, "cur.setCurrentCamera(orender)\n");
	fprintf(fid, "ipo = Blender.Ipo.New('Object','render_cam_objipo')\n");
	fprintf(fid, "orender.setIpo(ipo)\n");
	fprintf(fid, "locx = ipo.addCurve('LocX')\n");
	fprintf(fid, "locx.setInterpolation('Linear')\n");
	fprintf(fid, "locy = ipo.addCurve('LocY')\n");
	fprintf(fid, "locy.setInterpolation('Linear')\n");
	fprintf(fid, "locz = ipo.addCurve('LocZ')\n");
	fprintf(fid, "locz.setInterpolation('Linear')\n");
	fprintf(fid, "rotx = ipo.addCurve('RotX')\n");
	fprintf(fid, "rotx.setInterpolation('Linear')\n");
	fprintf(fid, "roty = ipo.addCurve('RotY')\n");
	fprintf(fid, "roty.setInterpolation('Linear')\n");
	fprintf(fid, "rotz = ipo.addCurve('RotZ')\n");
	fprintf(fid, "rotz.setInterpolation('Linear')\n");
	fprintf(fid, "camipo = Blender.Ipo.New('Camera','render_cam_camipo')\n");
	fprintf(fid, "crender.setIpo(camipo)\n");
	fprintf(fid, "lenscurve = camipo.addCurve('Lens')\n");
	fprintf(fid, "lenscurve.setInterpolation('Linear')\n");
	fprintf(fid, "\n");
	for (size_t i=0; i<cameras.size(); i++) {
		int camnum = ordering[i].second;
		EuclideanCamera* ec = ecam(camnum);
		int num = ec->get_frame()->get_orig_ind();
		assert(num == ordering[i].first);
		fprintf(fid, "locx.addBezier((%d,o%04d.LocX))\n", num, num);
		fprintf(fid, "locy.addBezier((%d,o%04d.LocY))\n", num, num);
		fprintf(fid, "locz.addBezier((%d,o%04d.LocZ))\n", num, num);
		fprintf(fid, "rotx.addBezier((%d,o%04d.RotX*18/3.141593))\n", num, num);
		fprintf(fid, "roty.addBezier((%d,o%04d.RotY*18/3.141593))\n", num, num);
		fprintf(fid, "rotz.addBezier((%d,o%04d.RotZ*18/3.141593))\n", num, num);
		fprintf(fid, "lenscurve.addBezier((%d,c%04d.lens))\n", num, num);
		fprintf(fid, "\n");
	}
	fprintf(fid, "\n");

	fprintf(fid, "allob=Object.Get()\n");
	fprintf(fid, "for o in allob:\n");
	fprintf(fid, "   print o.name[:10]\n");
	fprintf(fid, "   if o.name.startswith('libmv_cam'):\n");
	fprintf(fid, "	 cur.unlink(o)\n");
	fprintf(fid, "\n");

	fprintf(fid, "# Point cloud\n");
	fprintf(fid, "ob=Object.New('Mesh','libmv_point_cloud')\n");
	fprintf(fid, "ob.setLocation(0.0,0.0,0.0)\n");
	fprintf(fid, "mesh=ob.getData()\n");
	fprintf(fid, "cur.link(ob)\n");
	for (size_t i=0; i<structure.size(); i++) {
		EuclideanPointStructure *ep = (EuclideanPointStructure*)structure[i];
		fprintf(fid, "v = NMesh.Vert(%g,%g,%g)\n", ep->X[0], ep->X[1], ep->X[2]);
		fprintf(fid, "mesh.verts.append(v)\n");
	}
	fprintf(fid, "mesh.update()\n");
	fprintf(fid, "cur.update()\n\n");

	fprintf(fid, "# Add a helper object to help manipulating joined camera and points\n");
	fprintf(fid, "scene_dummy = Object.New('Empty','libmv_scene')\n");
	fprintf(fid, "scene_dummy.setLocation(0.0,0.0,0.0) \n");
	fprintf(fid, "cur.link(scene_dummy)\n");
	// FIXME orender is from IPO... do IPO curves!
//	fprintf(fid, "scene_dummy.makeParent([orender,ob])\n");
	fprintf(fid, "scene_dummy.makeParent([ob])\n");
	for (size_t i=0; i<cameras.size(); i++) {
		int camnum = ordering[i].second;
		EuclideanCamera* ec = ecam(camnum);
		int num = ec->get_frame()->get_orig_ind();
		assert(num == ordering[i].first);
		fprintf(fid, "scene_dummy.makeParent([o%04d])\n", num);
	}
	fprintf(fid, "\n");
//	fprintf(fid, "scene_dummy.setEuler((-3.141593/2, 0.0, 0.0))\n");
	fprintf(fid, "scene_dummy.SizeX=1.0\n");
	fprintf(fid, "scene_dummy.SizeY=1.0\n");
	fprintf(fid, "scene_dummy.SizeZ=1.0\n");

	fclose(fid);
}

/* force w > 0 for all points */
void force_positive_depth(ProjectiveReconstruction *pr)
{
	std::vector<size_t> done_cameras;
	std::vector<size_t> done_points;
	done_cameras.resize(pr->cameras.size());
	done_points.resize(pr->structure.size());

	size_t numcams_done = 0;
	size_t numpoints_done = 0;

	const size_t NEW = 0;
	const size_t QUEUED = 1;
	const size_t DONE = 2;

	for (size_t i=0; i<done_cameras.size(); i++) 
		done_cameras[i] = NEW;
	for (size_t i=0; i<done_points.size(); i++) 
		done_points[i] = NEW;

	std::vector<size_t> point_queue;
	std::vector<size_t> camera_queue;

	// Initial state which should cascade into fixing all cameras and points
	size_t start_cam=0;
	done_cameras[start_cam] = QUEUED;
	camera_queue.push_back(start_cam);

	std::multimap<Camera*, Structure*>::iterator p;
	std::pair<std::multimap<Camera*, Structure*>::iterator,
		  std::multimap<Camera*, Structure*>::iterator >
			  camera_range;

	std::multimap<Structure*, Camera*>::iterator c;
	std::pair<std::multimap<Structure*, Camera*>::iterator,
		  std::multimap<Structure*, Camera*>::iterator >
			  point_range;

	int count=0;
	while (numcams_done < pr->cameras.size()) {
//		printf("numcamsdone=%d, numpointsdone=%d\n",numcams_done, numpoints_done);
//		printf(" camera_queue.size()=%d\n",camera_queue.size());
		while (camera_queue.size()) {
			size_t new_cam = camera_queue[camera_queue.size()-1];
			camera_queue.pop_back();
			assert(done_cameras[new_cam] == QUEUED);
			ProjectiveCamera* cam = pr->pcam(new_cam);
			camera_range = pr->cam_sees.equal_range(cam);
			for (p=camera_range.first;
			     p!=camera_range.second; ++p) {
				ProjectiveCamera* tcam = (ProjectiveCamera*) p->first;
				assert (cam==tcam);
				PointStructure* ss = (PointStructure*) p->second;
				size_t point_number = ss->get_number();
//				printf(" found structure=%d\n",point_number);
//				printf(" done=%d\n",done_points[point_number]);

				if (done_points[point_number] == NEW) {
//					printf(" not done-- pointnum=%d\n", point_number);
					vec t = dot(*cam, *ss);
					if (t[2] < 0)
						*ss *= -1;
					done_points[point_number] = QUEUED;
					point_queue.push_back(point_number);
				}
			}
			done_cameras[new_cam] = DONE;
			numcams_done++;
		}

//		printf(" point_queue.size()=%d\n",point_queue.size());
		while (point_queue.size()) {
			size_t new_point = point_queue[point_queue.size()-1];
			assert(done_points[new_point] == QUEUED);
			point_queue.pop_back();
			PointStructure* ss = (PointStructure*) pr->structure[new_point];

//			Track* tt = ss->get_track();
			int npts = 0;
//			printf("   track length for this point=%d\n",tt->size());

//			MeasurementVisitor mv(pr, new_point);
//			while (mv.next()) {
//				PointStructure* tss = (PointStructure*) mv.structure();
//				ProjectiveCamera* cam = (ProjectiveCamera*) mv.camera();
//
			point_range = pr->visible_in.equal_range(ss);
			for (c=point_range.first;
			     c!=point_range.second; ++c) {
				PointStructure* tss = (PointStructure*) c->first;
				ProjectiveCamera* cam = (ProjectiveCamera*) c->second;

				assert(tss == ss);
				size_t camera_number = cam->get_number();
//				printf(" found camera=%d\n",camera_number);
//				printf(" cam is done=%d\n",done_cameras[camera_number]);
				if (done_cameras[camera_number] == NEW) {
					printf(" not done-- camnum=%d\n", camera_number);
					vec t = dot(*cam, *ss);
					if (t[2] < 0)
						*cam *= -1;
					done_cameras[camera_number] = QUEUED;
					camera_queue.push_back(camera_number);
				}
				npts++;
			}
//			printf("   actual measurements=%d\n",npts);
			done_points[new_point] = DONE;
			numpoints_done++;
		}
		count++;
//		if (count > 15)
//			break;
//		getchar();
	}

	// SANITY!
	MeasurementVisitor mv(pr);
	while (mv.next()) {
		ProjectiveCamera* cam = (ProjectiveCamera*) mv.camera();
		PointStructure* ss = (PointStructure*) mv.structure();
		vec t = dot(*cam, *ss);
//		assert(t[2] > 0);
		if (t[2] <= 0) {
			fprintf(stderr, "Failed at forcing positive depth; t[2] = %g\n", t[2]);
		}
	}
}

void Reconstruction::drop_structure(std::set<Structure*> &str)
{

	// Take only new structure
	std::vector<Structure*> tmp_str;
	size_t new_size = structure.size()-str.size();
	tmp_str.resize(structure.size()-str.size());
	size_t pos=0;
	for (size_t i=0; i<structure.size(); i++) {
		if (str.count(structure[i]) == 0) {
			tmp_str[pos++] = structure[i];
		}
	}
	structure.resize(new_size);
	for (size_t i=0; i<new_size; i++) {
		structure[i] = tmp_str[i];
	}

	// Empty visibility information (not axiomatic data)
	visible_in.clear();
	cam_sees.clear();

	// Take only new measurements
	std::map<CToS, Feature*> tmp_measurements;
	std::map<CToS, Feature*>::iterator m;
	for (m=measurements.begin(); m!=measurements.end(); ++m) {
		CToS ctos = m->first;
		Camera* cam = ctos.first;
		Structure* st = ctos.second;
		if (str.count(st) == 0) {
			tmp_measurements[ctos] = m->second;
			cam_sees.insert(ctos);
			visible_in.insert(SToC(st, cam));
		}
	}
	measurements.clear();
	for (m=tmp_measurements.begin(); m!=tmp_measurements.end(); ++m) {
		measurements[m->first] = m->second;
	}
}

void ProjectiveReconstruction::trim_outliers()
{
	size_t i=0;
	size_t n = cam_sees.size();
	size_t original_number_points = structure.size();
	vec resids(n);
	vec dropped(n);
	fill(dropped, 0.);
	std::vector<size_t> which_structure;
	which_structure.resize(n);

	MeasurementVisitor mv(this);
	while (mv.next()) {
		double err = mv.camera()->reprojection_error(
				mv.structure(),
				mv.measurement());
		resids[i] = sqrt(err);
		which_structure[i] = mv.structure()->get_number();
		i++;
	}

	// KNOB KNOB
	RayleighDistribution r_inliers(SIGMA2_INLIERS);
	RayleighDistribution r_outliers(SIGMA2_OUTLIERS);

//	dpmat(resids);

	double mean = sum(resids) / resids.size();
	resids /= mean;
	double lp = em_fit(resids, &r_inliers, &r_outliers, dropped);
	resids *= mean;
	r_inliers.sigma2 *= mean*mean;
	r_outliers.sigma2 *= mean*mean;

	double sigma2i = r_inliers.sigma2;
	double alpha = r_inliers.alpha;
	double sigma2o = r_outliers.sigma2;
	double err_thresh = sqrt(2*log(alpha * sigma2o / (1.-alpha)/sigma2i)
				 / (1./sigma2i - 1./sigma2o));

	// Our saftey factor can be much larger because we already went through
	// one level of outlier rejection. Mainly we want to eliminate really
	// nasty outliers.
	// KNOB
	double saftey_factor = 2;
	err_thresh *= saftey_factor;

	// Hack. Sometimes the fit is really good, so the threshhold is set very
	// low. That's bad. 
	// KNOB
	if (err_thresh < 2.0)
		err_thresh = 2.0;
	// KNOB
	if (err_thresh > 5.0)
		err_thresh = 5.0;

	number_items();
#if 1
	// Now drop measurements (not necessarily the actual points!) which are
	// badly in error.
	size_t orig_measurements = measurements.size();
	std::map<CToS, Feature*>::iterator it;
	std::map<CToS, Feature*>::iterator it_to_erase;
	for (it = measurements.begin();
			it != measurements.end(); ++it) {
		Camera *cam = it->first.first;
		Structure *str = it->first.second;
		Feature *ft = it->second;
		double err = cam->reprojection_error(str, ft); 
		if (err > err_thresh) {
			it_to_erase = it;  
			// we have to increment first because the iterator becomes invalid
			// after erasing.
			++it;

			// FIXME shift this into its own function
			measurements.erase(it_to_erase);

			// Now erase the record of it from visible_in
			bool erased_visible_in = false;
			std::multimap<Structure*,Camera*>::iterator p;
			std::pair<std::multimap<Structure*,Camera*>::iterator,
			          std::multimap<Structure*,Camera*>::iterator >
					  point_range;
			point_range = visible_in.equal_range(str);
			for (p = point_range.first; p != point_range.second; ++p) {
				if (p->second == cam) {
					assert (p->first == str);
					visible_in.erase(p);
					erased_visible_in = true;
					break;
				}
			}
			assert (erased_visible_in);

			// Check if this is the last visible point
			// FIXME really if there is less than about 4 views of the point we
			// should drop it. Oh well.
			point_range = visible_in.equal_range(str);
			if (point_range.first == point_range.second) {
				// delete the structure
				structure.erase(structure.begin()+str->get_number());
				// this is potentially n^2, but since it's unlikely, it's probably ok.
				number_items(); 
				delete str;
			}


			// Now erase the record of it from cam_sees
			bool erased_cam_sees = false;
			std::multimap<Camera*,Structure*>::iterator c;
			std::pair<std::multimap<Camera*,Structure*>::iterator,
			          std::multimap<Camera*,Structure*>::iterator >
					  cam_range;
			cam_range = cam_sees.equal_range(cam);
			for (c = cam_range.first; c != cam_range.second; ++c) {
				if (c->second == str) {
					assert (c->first == cam);
					cam_sees.erase(c);
					erased_cam_sees = true;
					break;
				}
			}
			assert (erased_cam_sees);
		}
	}
	printf("  __________________________________________\n");
	printf("/'\n");
	printf("   kept=%d/%d, thresh=%g\n",
			measurements.size(),orig_measurements, err_thresh);
	printf("\\.__________________________________________\n");

	return;
#else

	// old version here
	std::set<Structure*> drops;
	for (i=0; i<n; i++) {
		if (resids[i] > err_thresh)
			drops.insert(structure[which_structure[i]]);
	}
	drop_structure(drops);

	printf("  __________________________________________\n");
	printf("/'\n");
	printf("   deleted=%d/%d, dropped=%g, lp=%g, thresh=%g\n",
			drops.size(),original_number_points,sum(dropped), lp,
			err_thresh);
	printf("   sigma2i=%g, sigma2o=%g, alphai=%g\n", sigma2i, sigma2o, alpha);
	printf("\\.__________________________________________\n");
#endif
}

void dump_many_reconstructions(
		const char* fn,
		std::vector<ProjectiveReconstruction*> &reconstructions)
{
	FILE* fid = fopen(fn, "wb");
	assert(fid);

	size_t size = reconstructions.size();
	fwrite(&size, sizeof(size_t), 1, fid);

	for (size_t i=0; i<reconstructions.size(); i++)
		dump_reconstruction_to_file(*(reconstructions[i]), fid);

	fclose(fid);
}

void read_many_reconstructions(
		TrackedSequence& ts,
		const char *fn,
		std::vector<ProjectiveReconstruction*> *reconstructions)
{
	FILE* fid = fopen(fn, "rb");
	assert(fid);

	size_t size;
	fread(&size, sizeof(size_t), 1, fid);
	reconstructions->resize(size);
	assert(size < 10000);

	for (size_t i=0; i<size; i++) {
		ProjectiveReconstruction *pr;
		pr = read_reconstruction_from_file(ts, fid);
		(*reconstructions)[i] = pr;
	}

	fclose(fid);
}

// FIXME euclidean test/save not tested at all
void dump_euclidean_reconstruction_to_file(EuclideanReconstruction& pr, const char *fn)
{
	FILE* fid = fopen(fn, "wb");
	assert(fid);
	dump_euclidean_reconstruction_to_file(pr, fid);
	fclose(fid);
}

void dump_euclidean_reconstruction_to_file(EuclideanReconstruction& pr, FILE* fid) 
{
	// 4-byte                 ncams
	// 4-byte                 npoints
	// 4-byte                 nmeasurements
	// 8-byte*12*ncams        cameras
	// 8-byte*4*npts          points
	// cam#/str#/frame#/feat# measurements

	// need consistent numbering
	pr.number_items();

	// File header (ncams,npoints,nmeasurements)
	size_t ncams = pr.cameras.size();
	fwrite(&ncams, sizeof(size_t), 1, fid);
	size_t npoints = pr.structure.size();
	fwrite(&npoints, sizeof(size_t), 1, fid);
	size_t nmeasurements = pr.measurements.size();
	fwrite(&nmeasurements, sizeof(size_t), 1, fid);

	// Cameras
	for (size_t i=0; i<ncams; i++) {
		EuclideanCamera* ec = pr.ecam(i);
		size_t frame_number = ec->get_frame_number();
		fwrite(&frame_number, sizeof(size_t), 1, fid);
		double data[9+9+3];
		double *t = data;
		for (int j=0; j<3; j++) 
			for (int k=0; k<3; k++) 
				*(t++) = ec->R(j,k);
		for (int j=0; j<3; j++) 
			for (int k=0; k<3; k++) 
				*(t++) = ec->K(j,k);
		for (int j=0; j<3; j++) 
			*(t++) = ec->t[j];
		fwrite(data, sizeof(double), 9+9+3, fid);
	}

	// Structure
	for (size_t i=0; i<npoints; i++) {
		EuclideanPointStructure* ps = (EuclideanPointStructure*)pr.structure[i];
		double data[3];
		for (int k=0; k<3; k++) 
			data[k] = ps->X[k];
		fwrite(data, sizeof(double), 3, fid);
	}

	// Measurements
	MeasurementVisitor mv(&pr);
	while (mv.next()) {

		size_t tmp;

		tmp = mv.camera()->number;
		fwrite(&tmp, sizeof(int), 1, fid);

		tmp = mv.structure()->number;
		fwrite(&tmp, sizeof(int), 1, fid);

		// Frame number and feature number in that frame
		tmp = mv.measurement()->get_frame_number();
		fwrite(&tmp, sizeof(int), 1, fid);
		tmp = mv.measurement()->get_number();
		fwrite(&tmp, sizeof(int), 1, fid);
	}
}

EuclideanReconstruction *read_euclidean_reconstruction_from_file(TrackedSequence& ts, const char *fn) 
{
	FILE* fid = fopen(fn, "rb");
	assert(fid);
	EuclideanReconstruction *pr = read_euclidean_reconstruction_from_file(ts, fid);
	fclose(fid);
	return pr;
}

EuclideanReconstruction *read_euclidean_reconstruction_from_file(TrackedSequence& ts, FILE* fid)
{
	EuclideanReconstruction *pr = new EuclideanReconstruction;

	// File header (ncams,npoints,nmeasurements)
	size_t ncams;
	fread(&ncams, sizeof(size_t), 1, fid);
	size_t npoints;
	fread(&npoints, sizeof(size_t), 1, fid);
	size_t nmeasurements;
	fread(&nmeasurements, sizeof(size_t), 1, fid);

	pr->cameras.resize(ncams);
	pr->structure.resize(npoints);

	// Cameras
	for (size_t i=0; i<ncams; i++) {
		EuclideanCamera* pc = new EuclideanCamera;
		// FIXME there is a way to get at the raw matrix data...
		// somehow...
		size_t frame_number;
		fread(&frame_number, sizeof(size_t), 1, fid);
		assert(frame_number < ts.size());
		double data[9+9+3];
		double *t = data;
		fread(data, sizeof(double), 9+9+3, fid);
		for (int j=0; j<3; j++) 
			for (int k=0; k<3; k++) 
				pc->R(j,k) = *(t++);
		for (int j=0; j<3; j++) 
			for (int k=0; k<3; k++) 
				pc->K(j,k) = *(t++);;
		for (int j=0; j<3; j++) 
			 pc->t[j] = *(t++);;
		pc->set_number(i);
		pc->set_frame(ts[frame_number]);
		pr->cameras[i] = pc;
	}

	// Structure
	for (size_t i=0; i<npoints; i++) {
		EuclideanPointStructure* ps = new EuclideanPointStructure;
		double data[3];
		fread(data, sizeof(double), 3, fid);
		for (int k=0; k<3; k++) 
			ps->X[k] = data[k];
		ps->set_number(i);
		pr->structure[i] = ps;
	}

	// Measurements
	for (size_t i=0; i<nmeasurements; i++) {
		size_t cam_number;

		fread(&cam_number, sizeof(size_t), 1, fid);

		size_t point_number;
		fread(&point_number, sizeof(size_t), 1, fid);

		size_t frame_number;
		fread(&frame_number, sizeof(size_t), 1, fid);

		size_t feature_number;
		fread(&feature_number, sizeof(size_t), 1, fid);

		assert(cam_number < ncams);
		assert(point_number < npoints);
		assert(frame_number < ts.size());
		assert(feature_number < ts[frame_number]->size());
		Frame* fr = ts[frame_number];
		pr->push_measurement(pr->cameras[cam_number],
		                     pr->structure[point_number],
		                     (*fr)[feature_number]);
		pr->structure[point_number]->set_track(
				(*fr)[feature_number]->get_track());
	}

	return pr;
}

//======================================================================
//======================================================================
//======================================================================

} // namespace mv

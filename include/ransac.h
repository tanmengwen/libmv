#ifndef _RANSAC_H
#define _RANSAC_H

#include "mv.h"
#include "em_fit.h"

//======================================================================
//================= Robust Fitting functionality =======================
//======================================================================

namespace mv {

// FIXME really, this is a bad design. ransac fitters should use composition;
// each fitter inherits from an interface class, which is then passed as an
// abstract pointer to one of the ransac drivers.
// FIXME if the 'kernels' obeyed an abstract base class too, it would make
// mixing and matching very easy to try out.
template<class FitType, class SampleType>
class SuperRANSACDriver
{
public:
	bool search(int iterations=3)
	{
		if (all_samples.size() < min_samples)
			assert(0);
		best_score = HUGE_VAL;
		size_t num_improvements_found = 0;
		print_header();
		while (iterations--) {
			// FIXME add risk calculation based on outlier prior rather than a
			// fixed number of iterations.
			pick_subset();
			fit_subset(random_sample);
			dropped = random_dropped;

			foreach(FitType fit, random_fits) {
				calculate_residuals(fit);
				double new_score = calculate_score(residuals, iterations, &dropped);
				if (new_score < best_score) {
					num_improvements_found++;
					best_fit = fit;
					best_score = new_score;
					best_dropped = dropped;
					best_random_dropped = random_dropped;
					best_residuals = residuals;
					new_best_fit(new_score, iterations);
				}
			}
		}

		if (num_improvements_found == 0) {
			failed = true;
			printf("******************** FAILED AT RANSAC!\n");
			return false;
		}

		double err_thresh = calculate_error_threshold();
		this->error_thresh_ = err_thresh;
		assert(this->error_thresh_ > 0.0);

		int ns=0;
		for (size_t i=0; i<all_samples.size(); i++) {
			if (best_dropped[i] == 1.0)
				continue;

			if (best_residuals[i] < err_thresh) {
				add_sample_to_best_fit(all_samples[i]);
				ns++;
			} else {
				sample_is_not_in_best_fit(all_samples[i]);
			}
		}
		final_fit(inlier_samples);
		fprintf(stderr, "=========== ADDED %5d/%5d SAMPLES ==========\n",ns,all_samples.size());
		fprintf(stderr, "=========== THRESHOLD: %5.2g =================\n",err_thresh);

		return true;
	}

	void pick_subset()
	{
		int n = min_samples;
		int low = 0;
		int high = all_samples.size();

		int out[n];

		int i=0,j;
		while (i<n) {
			int k = low + rand() % (low-high);
			int dupe=0;
			for (j=0; j<i; j++) {
				if (out[j] == k) {
					dupe=1;
					break;
				}
			}
			if (dupe)
				continue;
			out[i++] = k;
		}

		random_dropped.resize(high);
		fill(random_dropped, 0.0);
		for (i=0; i<n; i++) {
			random_sample[i] = all_samples[out[i]];
			random_dropped[out[i]] = 1.0;
		}
	}

	void set_fit_size(size_t min_samples)
	{
		this->min_samples = min_samples;
		random_sample.resize(min_samples);
	}

	void set_min_samples(size_t minsamps) {
		min_samples = minsamps;
		random_sample.resize(minsamps);
	}

	// These must be implemented by a subclass
	virtual void fit_subset(std::vector<SampleType> &subset) = 0;
	virtual void calculate_residuals(FitType &fit) = 0;
	virtual double calculate_error_threshold() {return HUGE_VAL;};

	virtual void print_header() {};
	virtual void new_best_fit(double new_best_score, size_t iteration) {
		(void) new_best_score;
		(void) iteration;
	};

	virtual void final_fit(std::vector<SampleType> &subset) {
		fit_subset(subset);
		best_fit = random_fits[0];
	};
	virtual double calculate_score(const vec &residuals, size_t iteration, vec *dropped) {
		(void) iteration;
		(void) dropped;
		return sum(residuals)/residuals.size();
	}

	virtual void add_sample_to_best_fit(SampleType st) {
		inlier_samples.push_back(st);
	}
	virtual void sample_is_not_in_best_fit(SampleType st) {
		(void) st;
	}

	virtual ~SuperRANSACDriver() {};

	FitType best_fit;
	double best_score;
	std::vector<SampleType> all_samples;
	std::vector<FitType> random_fits;
	vec residuals;

	double error_thresh_;

	vec best_residuals;
private:
	bool failed;
	vec dropped;

	vec best_dropped;
	vec best_random_dropped;

	size_t min_samples;
	std::vector<SampleType> random_sample;
	std::vector<SampleType> best_sample;
	vec random_dropped;
	std::vector<SampleType> inlier_samples;

};

// KNOB
#define SIGMA2_INLIERS 0.1
// KNOB
#define SIGMA2_OUTLIERS 10.0
// KNOB
#define MINIMUM_INLIERS 0.5
// KNOB
#define MAXIMUM_DROP_FRACTION 0.5
template<class FitType, class SampleType>
class RANSACDriver : public SuperRANSACDriver<FitType, SampleType>
{
public:
	void print_header() {
		fprintf(stderr, "iter    logprob    sigma2I    sigma2O     alphaI     alphaO    dropped\n");
	}
	virtual double calculate_score(const vec &residuals, size_t iteration, vec *dropped) {
		double mean = sum(residuals)/residuals.size();
		r_inliers.sigma2 = SIGMA2_INLIERS;
		r_outliers.sigma2 = SIGMA2_OUTLIERS;
//		RayleighDistribution r_inliers(SIGMA2_INLIERS);
//		RayleighDistribution r_outliers(SIGMA2_OUTLIERS);
//		RayleighDistribution r_inliers(mean/100.);
//		RayleighDistribution r_outliers(mean);

		vec normed_residuals = residuals / mean;
		double lp = em_fit(normed_residuals, &r_inliers, &r_outliers, *dropped);
		double new_score = HUGE_VAL;
		r_inliers.sigma2 *= mean*mean;
		r_outliers.sigma2 *= mean*mean;
//		fprintf(stderr, "mean=%g, sigma2=%g\n",mean, r_inliers.sigma2);

		if (r_inliers.alpha > MINIMUM_INLIERS
//				&& r_inliers.sigma2 < 5 // FIXME KNOB
				&& (sum(*dropped) / (double) dropped->size() 
					< MAXIMUM_DROP_FRACTION))
//			new_score = -lp;
			new_score = r_inliers.sigma2;

		// debug
		fprintf(stderr, "%4d %10g %10g %10g %10g %10g %10g/%d XXXX lp=%g mean=%g \n",
				iteration,
				new_score,
				r_inliers.sigma2,
				r_outliers.sigma2,
				r_inliers.alpha,
				r_outliers.alpha,
				sum(*dropped), dropped->size(), lp, mean);
		return new_score;
	}

	virtual void new_best_fit(double new_best_score, size_t iteration) {
		best_inliers.sigma2 = r_inliers.sigma2;
		best_inliers.alpha = r_inliers.alpha;
		best_outliers.sigma2 = r_outliers.sigma2;
		best_outliers.alpha = r_outliers.alpha;
		fprintf(stderr, "%4d %10g %10g %10g %10g %10g\n",
				iteration,
				new_best_score,
				best_inliers.sigma2,
				best_outliers.sigma2,
				best_inliers.alpha,
				best_outliers.alpha);
	}

	virtual double calculate_error_threshold() {
		double sigma2i = best_inliers.sigma2;
		double alpha = best_inliers.alpha;
		double sigma2o = best_outliers.sigma2;
		double error_thresh = sqrt(2*log(alpha * sigma2o / (1.-alpha)/sigma2i)
					 / (1./sigma2i - 1./sigma2o));

		// KNOB!
		double saftey_factor = 0.75;
		error_thresh *= saftey_factor;
		return error_thresh;
	}

	virtual ~RANSACDriver() {};

	RayleighDistribution r_inliers;
	RayleighDistribution r_outliers;
	RayleighDistribution best_inliers;
	RayleighDistribution best_outliers;

private:
};

// KNOB
#define MLESAC_THRESH 1.4

template<class FitType, class SampleType>
class MLESACDriver : public SuperRANSACDriver<FitType, SampleType>
{
public:
	void print_header() {
		fprintf(stderr, "iter    score    alphaO    dropped\n");
	}
	virtual double calculate_score(const vec &residuals, size_t iteration, vec *dropped) {
		(void) dropped;
		(void) iteration;
		// KNOB
		double threshold = MLESAC_THRESH;

		double score = 0;
		for (size_t i=0; i<residuals.size(); i++) {
			if (residuals[i] < threshold)
				score += residuals[i];
			else 
				score += threshold;
		}

		return score/residuals.size();
	}

	virtual void new_best_fit(double new_best_score, size_t iteration) {
		fprintf(stderr, "iteration=%04d new best: %g\n", iteration, new_best_score);
	}

	virtual double calculate_error_threshold() {
		return MLESAC_THRESH;
	};
	virtual ~MLESACDriver() {};

private:
};

template<class FitType, class SampleType>
class AverageRANSACDriver : public SuperRANSACDriver<FitType, SampleType>
{
public:
	void print_header() {
		fprintf(stderr, "iter    mean\n");
	}
	virtual double calculate_score(const vec &residuals, size_t iteration, vec *dropped) {
		(void) dropped;
		double mean = sum(residuals)/residuals.size();
		fprintf(stderr, "%4d %10g\n", iteration, mean);
		return mean;
	}

	virtual ~AverageRANSACDriver() {};
};

template<class FitType, class SampleType>
class LMedSRANSACDriver : public SuperRANSACDriver<FitType, SampleType>
{
public:
	void print_header() {
		fprintf(stderr, "iter    LMedS\n");
	}
	virtual double calculate_score(const vec &residuals, size_t iteration, vec *dropped) {
		(void) dropped;
		(void) iteration;
		vec myresids(residuals);
		std::sort(myresids.begin(),myresids.end());
		double median = myresids[myresids.size()/2];
//		fprintf(stderr, "%4d %10g\n", iteration, median);
		return median;
	}

	virtual void new_best_fit(double new_best_score, size_t iteration) {
		fprintf(stderr, "iteration=%04d new best med sq: %g\n", iteration, new_best_score);
	}

	virtual ~LMedSRANSACDriver() {};
};

class ThreeViewSixPointRANSACDriver
//	: public RANSACDriver<ProjectiveReconstruction, ThreeFrameTrack >
//	: public MLESACDriver<ProjectiveReconstruction, ThreeFrameTrack >
	: public LMedSRANSACDriver<ProjectiveReconstruction, ThreeFrameTrack >
{
public:
	ThreeViewSixPointRANSACDriver(Frame *frames[3]);
	ThreeViewSixPointRANSACDriver(TrackedSequence &ts, size_t first);
	ThreeViewSixPointRANSACDriver(TrackedSequence &ts, size_t first, size_t middle, size_t last);
	void prepare_samples();
	void fit_subset(std::vector<ThreeFrameTrack> &subset);
	void final_fit(std::vector<ThreeFrameTrack> &subset);
//	double score_fit(ProjectiveReconstruction &pr);
	void calculate_residuals(ProjectiveReconstruction &pr);
	void add_sample_to_best_fit(ThreeFrameTrack tft);
	void sample_is_not_in_best_fit(ThreeFrameTrack tft);

private:
	Frame *frames[3];
	TrackedSequence *ts;
};

typedef std::pair<PointStructure*,PointStructure*> PointPair;


class ProjectiveAlignmentRANSACDriver
//	: public AverageRANSACDriver<mat, PointPair>
	: public LMedSRANSACDriver<mat, PointPair >
{
public:
	ProjectiveAlignmentRANSACDriver(ProjectiveReconstruction *A, ProjectiveReconstruction *B);
	void fit_subset(std::vector<PointPair> &subset);
	void calculate_residuals(mat &H);
	void final_fit(std::vector<PointPair> &subset);
	void transform_cost(double *p, double *hx, int m, int n, void *adata);

private:
	mat PBplusPA;
	mat PA, PB;
	vec h;

	size_t othercam_number;

	ProjectiveReconstruction *A;
	ProjectiveReconstruction *B;
};


} // namespace mv
#endif // _RANSAC_H

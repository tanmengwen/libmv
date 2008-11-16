#ifndef _UNSCENTED_H
#define _UNSCENTED_H

#include "mv.h"

namespace mv {

// FIXME: Perhaps the API should be made clearer so that accessing mean /
// covariance is explicitly transformed_mean and transformed_variance, and the
// old values are kept around.
class UnscentedTransform
{
public:
	UnscentedTransform(
			// length n vector
			double *mean,
			// n*n row-major symmetric covariance matrix
			double *sigma2,
			size_t n,
			// Scaled Unscented Transform parameters
			double k,
			// Controls the spread of the points
			double alpha,
			// Should be 2 for Gaussian priors
			double beta)
		: mean_(mean), covariance_(sigma2), n_(n),
		  k_(k), alpha_(alpha), beta_(beta) {
			lambda_ = alpha*alpha*(n-k) - n;
			generate_samples();
	}
	size_t samples() {
		return 2*n_ + 1;
	}
	double *sample(size_t i) {
		return &(samples_(i,0));
	}
	void compute_transformed_moments() {
		double w_mean = lambda_ / (n_ + lambda_);
		double w_cov = lambda_ / (n_ + lambda_) + 1 - alpha_*alpha_ + beta_;
		double w_other = 1. / (2.*(n_+lambda_));

		// mean
		for (size_t j=0; j<n_; j++)
			mean_[j] = w_mean * samples_(0,j);
		for (size_t i = 1; i<2*n_+1; i++)
			for (size_t j=0; j<n_; j++)
				mean_[j] += w_other * samples_(i,j);
		
		// covariance
		mat cov_new(n_,n_);
		zero(cov_new);
		vec tmp(n_);
		for (size_t i = 0; i<2*n_+1; i++) {
			for (size_t j=0; j<n_; j++)
				tmp(j) = samples_(i, j) - mean_[j];
			if (i == 0) {
				cov_new += w_cov * outer_prod(tmp,tmp);
			} else {
				cov_new += w_other * outer_prod(tmp,tmp);
			}
		}
		cov_new_ = cov_new;
	}
	double *covariance() {return cov_new_.data().begin();};
	double *mean() {return mean_;}
private:
	void generate_samples() {
		mat cov(n_,n_);
		fill(cov,covariance_);
		fmat scaled = (n_+lambda_)*cov;

		// Poor man's matrix square root
//		dpmat(cov);
//		dpmat(scaled);
		bmat s;
		fmat U, Vt;
		s = svd(scaled, U, Vt);
		for (size_t i=0; i<n_; i++)
			s(i,i) = sqrt(s(i,i));
		mat sqrt_scaled = dot(U, s);
		sqrt_scaled = dot(sqrt_scaled, Vt);
//		dpmat(scaled);
//		dpmat(sqrt_scaled);
//		sqrt_scaled = inv(sqrt_scaled);
//		printf("inverted!\n");
//		dpmat(sqrt_scaled);

		samples_.resize(2*n_+1,n_);
		for (size_t j=0; j<n_; j++) {
			samples_(0, j) = mean_[j];
		}
		for (size_t i=1; i<n_+1; i++) {
			for (size_t j=0; j<n_; j++) {
				samples_(i, j) = mean_[j] + sqrt_scaled(i-1, j);
			}
			for (size_t j=0; j<n_; j++) {
				samples_(n_+i, j) = mean_[j] - sqrt_scaled(i-1, j);
			}
		}
//		dpmat(samples_);
	}
	double *mean_;
	double *covariance_;
	size_t n_;
	double k_;
	double alpha_;
	double beta_;

	double lambda_;
	mat samples_;
	mat cov_new_;
};

} // namespace mv
#endif // _UNSCENTED_H

#include <mv.h>
#include "ransac.h"
#include "assert.h"

namespace mv {

// see page 179 of hartley & zisserman
// FIXME make normalization optional
void camera_resection(const mat &x_orig, const mat &X, mat *P)
{
	size_t N = x_orig.size2();

	// Normalize the points
	mat x_homogenous(3,N);
	for (size_t i=0; i<N; i++) {
		x_homogenous(0,i) = x_orig(0,i);
		x_homogenous(1,i) = x_orig(1,i);
		x_homogenous(2,i) = 1.0;
	}

//	mat H = normalizing_homography(x_homogenous);
//	mat Hinv = inv(H);
//	mat x = dot(H, x_homogenous);
	mat x = x_homogenous;

	assert(X.size2() == N);
	fmat design(2*N-1, 12);
	for (size_t i=0; i<N; i++) {
		for (size_t k=0; k<4; k++) {
			// first equation for pt i
			design(2*i+0, 0+k) = 0.;
			design(2*i+0, 4+k) =-x(2,i)*X(k,i);
			design(2*i+0, 8+k) = x(1,i)*X(k,i);
//			printf("i=%d, x(2,i)=%g\n",i,x(2,i));

			// second equation for pt i
			if (i != N-1) {
				design(2*i+1, 0+k) =  x(2,i)*X(k,i);
				design(2*i+1, 4+k) =  0.;
				design(2*i+1, 8+k) = -x(0,i)*X(k,i);
			}
		}
	}

	vec p;
	nullspace(design, p);

//	dpmat(p);
	mat HP = reshape(p, 3,4);
//	mat tmp = dot(Hinv, HP);
	mat tmp = HP;
	*P = tmp;
//	dpmat(*P);
}


// Standard notation: x = P*X
// this solves for P in the presence of outliers

class ResectionRANSACDriver
//	: public RANSACDriver<mat, int>
//	: public MLESACDriver<mat, int>
	: public LMedSRANSACDriver<mat, int>
{
public:
	ResectionRANSACDriver(const mat& x, const mat& X);
	void fit_subset(std::vector<int> &subset);
	void calculate_residuals(mat &P);
	virtual double calculate_error_threshold() {
		vec myresids(best_residuals);
		std::sort(myresids.begin(),myresids.end());
		double median = myresids[myresids.size()/2];
//		dpmat(myresids);
		// KNOB
		return 2*median;
	}

private:
	const mat &x;
	const mat &X;
};

ResectionRANSACDriver::ResectionRANSACDriver(const mat& xx, const mat& XX)
	: x(xx), X(XX)
{
	size_t n = XX.size2();
	all_samples.resize(n);
	residuals.resize(n);
	for (size_t i=0; i<n; i++)
		all_samples[i] = i;
	random_fits.resize(1);
	set_min_samples(6);
}

void ResectionRANSACDriver::fit_subset(std::vector<int> &subset)
{
	int n = subset.size();
	mat xx(2,n);
	mat XX(4,n);

	for (int i=0; i<n; i++) {
		for (int j=0; j<2; j++)
			xx(j,i) = x(j,subset[i]);
		for (int j=0; j<4; j++)
			XX(j,i) = X(j,subset[i]);
	}

	random_fits.resize(1);
	camera_resection(xx,XX, &(random_fits[0]));
}

void ResectionRANSACDriver::calculate_residuals(mat &P)
{
	mat xxp = dot(P, X);
	for (size_t i=0; i<X.size2(); i++) {
		double ex = xxp(0,i)/xxp(2,i) - x(0,i);
		double ey = xxp(1,i)/xxp(2,i) - x(1,i);
		residuals[i] = sqrt(ex*ex + ey*ey);
	}
//	dpmat(residuals);
}

bool robust_camera_resection(const mat& x, const mat& X, mat *P, double *score)
{
	ResectionRANSACDriver driver(x, X);
	// KNOB
	if (!driver.search(200)) {
		return false;
	}
	*P = driver.best_fit;
	*score = driver.best_score;
	return true;
}

} // namespace mv

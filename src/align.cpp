#include "mv.h"
#include "ransac.h"

namespace mv {

/*
 * finds T such that A = T*B where T is [sR t; 0 1] (4x4)
 * A is (3xn) many points, each as col vectors
 * B is (3xn) many points, each as col vectors
 */
void find_aligning_transform(const mat& A, const mat& B, mat *T)
{
	assert(A.size1() == B.size1());
	assert(A.size2() == B.size2());

	int n = A.size2();

	// find first moment (mean)
	vec mean_a(sum2(A));
	vec mean_b(sum2(B));
	mean_a /= n;
	mean_b /= n;
	assert(mean_a.size() == 3);
	assert(mean_b.size() == 3);

	mat AA(A);
	mat BB(B);

	// subtract mean
	for (int i=0; i<n; i++)
		for (int j=0; j<3; j++)
			AA(j,i) -= mean_a[j];

	for (int i=0; i<n; i++)
		for (int j=0; j<3; j++)
			BB(j,i) -= mean_b[j];

	// find second moment
	double mean_mag_a=0.0, mean_mag_b=0.0;
	for (int i=0; i<n; i++) {
		double ma=0;
		for (int j=0; j<3; j++) {
			ma += AA(j,i)*AA(j,i);
		}
		mean_mag_a += sqrt(ma);

		double mb=0;
		for (int j=0; j<3; j++) {
			mb += BB(j,i)*BB(j,i);
		}
		mean_mag_b += sqrt(mb);
	}
	mean_mag_a /= n;
	mean_mag_b /= n;

	// normalize relative sizes
	AA /= mean_mag_a;
	BB /= mean_mag_b;

	// R = orthogonal polar factor of BB*AA' (procrustes)
	// min || AA - R*BB ||^2 s.t. R*R^T = 1
	// note that this is the transpose of what is often shown in textbooks
	mat design = dot(BB, transpose(AA));
	fmat U,Vt;
	bmat s = svd(design, U, Vt);
//	for (int i=0; i<3; i++)
//		printf("s(%d,%d)=%f\n", i,i,s(i,i));
	mat R = dot(U, Vt);

	double scale = mean_mag_a/mean_mag_b;
	mat Ta(eye(4));
	mat sR(eye(4));
	mat Tb(eye(4));

	for (int i=0; i<3; i++)
		Ta(i,3) = mean_a[i];
	for (int i=0; i<3; i++)
		Tb(i,3) = -mean_b[i];
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			sR(i,j) = scale*R(j,i); // note transpose

//	printf("mean_mag_a=%f\n", mean_mag_a);
//	printf("mean_mag_b=%f\n", mean_mag_b);
//	printf("scale=%f\n",scale);
//	dpmat(Ta);
//	dpmat(sR);
//	dpmat(Tb);

	mat tmp = dot(Ta, sR);
	*T = dot(tmp, Tb);
}

class PointCloudAlignmentRANSACDriver
	: public RANSACDriver<mat, int>
{
public:
	PointCloudAlignmentRANSACDriver(const mat& A, const mat& B);
	void fit_subset(std::vector<int> &subset);
	void calculate_residuals(mat &T);

private:
	const mat &A;
	const mat &B;
};

PointCloudAlignmentRANSACDriver::PointCloudAlignmentRANSACDriver(const mat& AA, const mat& BB)
	: A(AA), B(BB)
{
	size_t n = AA.size2();
	all_samples.resize(n);
	residuals.resize(n);
	for (size_t i=0; i<n; i++)
		all_samples[i] = i;
	random_fits.resize(1);
	// KNOB
	set_min_samples(4);
}

void PointCloudAlignmentRANSACDriver::fit_subset(std::vector<int> &subset)
{
	int n = subset.size();
	mat a(3,n);
	mat b(3,n);

	for (int i=0; i<n; i++) {
		for (int j=0; j<3; j++) {
			a(j,i) = A(j,subset[i]);
			b(j,i) = B(j,subset[i]);
		}
	}

//	dpmat(a);
//	dpmat(b);
	random_fits.resize(1);
	find_aligning_transform(a,b, &(random_fits[0]));
}

void PointCloudAlignmentRANSACDriver::calculate_residuals(mat &T)
{
	// note that this is no longer rayleigh distributed, but is actually
	// maxwell-boltzmann distributed. i don't have an ML estimator for the
	// parameterts yet though :(
	for (size_t i=0; i<A.size2(); i++) {
		vec ptA(4), ptB(4);
		ptA[0] = A(0,i);
		ptA[1] = A(1,i);
		ptA[2] = A(2,i);
		ptA[3] = 1.0;
		ptB[0] = B(0,i);
		ptB[1] = B(1,i);
		ptB[2] = B(2,i);
		ptB[3] = 1.0;
		ptB = dot(T, ptB);
		ptB /= ptB[3];
		residuals[i] = norm_2(ptA-ptB);
	}
//	dpmat(residuals);
}

bool robust_find_aligning_transform(const mat& A, const mat& B, mat *T, double *error_thresh)
{
	PointCloudAlignmentRANSACDriver driver(A, B);
//	dpmat(A);
	// KNOB
	if (!driver.search(20)) {
		return false;
	}
	*T = driver.best_fit;
	assert (driver.error_thresh_ > 0.0);
	*error_thresh = driver.error_thresh_;
	return true;
}

/*
 * finds T such that A = T*B where T is any 4x4 projectivity
 * A is (4xn) many points, each as col vectors
 * B is (4xn) many points, each as col vectors
 */
void find_aligning_projective_transform(const mat& A, const mat& B, mat *T)
{
	size_t n = A.size2();
	assert(B.size2() == n);

	fmat design(4*n, n + 4*4);
	fill(design, 0.0);
	for (size_t i=0; i<n; i++) {
		for (size_t j=0; j<4; j++) {
			design(4*i+j, i) = -A(j, i);
		}
		for (size_t j=0; j<4; j++) {
			for (size_t k=0; k<4; k++) {
				design(4*i+j, n + 4*j+k) = B(k, i);
			}
		}
	}
	vec t;
	nullspace(design, t);
	vec trimmed(subrange(t,n,n+16));

	// These transformations are only for testing
	double mag = norm_2(trimmed);
	if (trimmed[0] < 0)
		trimmed /= -mag;
	else
		trimmed /= mag;

	*T = reshape(trimmed, 4,4);
}

#if 0
static class ProjectiveAligner
	: public RANSACDriver<mat, PointPair>
{
public:
	ProjectiveAlignmentRANSACDriver(ProjectiveReconstruction *A, ProjectiveReconstruction *B);
	void fit_subset(std::vector<PointPair> &subset);
	void calculate_residuals(mat &H);

private:
	ProjectiveReconstruction *A;
	ProjectiveReconstruction *B;
};
ProjectiveAligner::ProjectiveAligner(
		ProjectiveReconstruction *A,
	   	ProjectiveReconstruction *B)
{
	foreach (Structure* pts, A->structure) {
		foreach (Structure* pts_other, B->structure) {
			if (pts->get_track_number() != pts_other->get_track_number())
				continue;

			PointStructure* ptA = (PointStructure*) pts;
			PointStructure* ptB = (PointStructure*) pts_other;
			all_samples.push_back(PointPair(ptA, ptB));
		}
	}

	A->number_items();
	B->number_items();
	this->A = A;
	this->B = B;

	set_min_samples(2);

	/* For now calculate error with replacement */
	residuals.resize(all_samples.size());
	random_fits.resize(1);
}

void ProjectiveAligner::fit_subset(std::vector<PointPair> &random_sample)
{
	std::vector<PointStructure*> XAs;
	std::vector<PointStructure*> XBs;
	for (size_t i=0; i<random_sample.size(); i++) {
		XAs.push_back(random_sample[i].first);
		XBs.push_back(random_sample[i].second);
	}
	random_fits[0] = mv::align_oneview(PBplusPA, h, XAs, XBs);
}

void ProjectiveAligner::calculate_residuals(mat &H)
{
	int i=0;
	mat Hinv = inv(H);
	foreach (PointPair pp, all_samples) {
		MeasurementVisitor mv(B, pp.second->number);
		residuals[i] = 0;
		while (mv.next()) {
			assert (pp.second->number == mv.structure()->number);
			if (mv.camera()->number != othercam_number)
				continue;
			// FIXME abstract this projection code!!!!
			PointStructure HX;
			vec hx = dot(H, *(pp.first));
			vec x = dot(*(ProjectiveCamera*)mv.camera(), hx);
			x /= x[2];
			PointFeature *x_measured = (PointFeature*) mv.measurement();
			double dx = x[0] - x_measured->x();
			double dy = x[1] - x_measured->y();
			double err = dx*dx+dy*dy;
			residuals[i] += sqrt(err);
		}
		i++;
	} 
}
#endif

} // namespace mv


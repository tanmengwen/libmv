#include <testmv.h>
#include "resection.h"
#include "test_resection_data.h"

TEST(simple_projective_resection) {
	MATDATA(x);
	MATDATA(X);
	MATDATA(P);
	// take the first 6 inliers; minimal case
	mat x_in = subrange(x, 0,2, 0,10);
	mat X_in = subrange(X, 0,4, 0,10);
	mat P_resect(3,4);
	camera_resection(x_in, X_in, &P_resect);
	if (P_resect(0,0) < 0. && (P(0,0) > 0.))
		P_resect *= -1;
	mat D = P - P_resect;
	AllSmall(D);
}

TEST(robust_projective_resection_with_no_outliers_works) {
	MATDATA(x);
	MATDATA(X);
	MATDATA(P);

	// The robust fitter BETTER work with no outliers!
	mat x_in = subrange(x, 0,2, 0,num_inliers);
	mat X_in = subrange(X, 0,4, 0,num_inliers);

	mat P_resect(3,4);
	double score;
	robust_camera_resection(x_in, X_in, &P_resect, &score);
	if (P_resect(0,0) < 0. && (P(0,0) > 0.))
		P_resect *= -1;
	mat D = P - P_resect;
	AllSmall(D);
}

TEST(robust_projective_resection) {
	MATDATA(x);
	MATDATA(X);
	MATDATA(P);

	mat P_resect(3,4);
	double score;
	robust_camera_resection(x, X, &P_resect, &score);

	if (P_resect(0,0) < 0. && (P(0,0) > 0.))
		P_resect *= -1;
	mat D = P - P_resect;
	AllSmall(D);
}

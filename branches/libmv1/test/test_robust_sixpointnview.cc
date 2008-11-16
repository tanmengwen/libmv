#include "testmv.h"
#include "robust_sixpointnview.h"
#include "test_robust_sixpointnview_data.h"

/*
TEST(ten_views_with_no_outliers_still_works) {
	MATDATA(x);
	double error_threshold;
	ProjectiveReconstruction* result = robust_sixpoint_n_view(
			x, &error_threshold);
	double min_rms = result->rms_reprojection_error();
	printf("rms=%g\n", min_rms);
	printf("mean=%g\n", result->mean_reprojection_error());
	printf("error_threshold=%g\n", error_threshold);
	Small(min_rms);
	result->delete_contents();
	delete result;
}
*/

TEST(ten_views_with_many_outliers_still_works) {
	MATDATA(x_noisy);
	double error_threshold;
	ProjectiveReconstruction* result = robust_sixpoint_n_view(
			x_noisy, &error_threshold);
	double min_rms = result->rms_reprojection_error();
	printf("rms=%g\n", min_rms);
	printf("mean=%g\n", result->mean_reprojection_error());
	printf("error_threshold=%g\n", error_threshold);
	Small(min_rms/10);
	result->bundle_adjust();
	result->delete_contents();
	delete result;
}

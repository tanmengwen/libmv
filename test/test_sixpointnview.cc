#include "testmv.h"
#include "sixpointnview.h"
#include "test_sixpointnview_data.h"

TEST(ten_views_perfect_data_leaves_tiny_rms) {
	MATDATA(x);
	ProjectiveReconstruction* result = six_point_n_view(x);
	double min_rms = result->rms_reprojection_error();
	printf("rms: %g\n", min_rms);
	printf("mean: %g\n", result->mean_reprojection_error());
	Small(min_rms);
	result->print_cameras();
//	Equals(6u, result->cameras.size());
//	Equals(6u, result->structure.size());
//	Equals(180, result->measurements.size());
	result->bundle_adjust();
	result->delete_contents();
	delete result;
}

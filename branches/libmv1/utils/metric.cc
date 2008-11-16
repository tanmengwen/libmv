#include <stdio.h>
#include "json_numeric.h"
#include "reconstruction.h"
#include "mv.h"
#include "mv_util.h"
#include "extern/google/gflags.h"
using namespace mv;

DEFINE_string(output,           "",  "Output metric reconstruction to this file");
DEFINE_string(reconstruction,   "",  "Projective reconstruction to upgrade to metric");
DEFINE_string(intrinsics,       "",  "Camera calibration information (focal length etc)");
DEFINE_bool  (trim_outliers, false,  "Trim outliers before bundling ");
DEFINE_bool  (force_positive, true,  "Force points to be in front of cameras");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"upgrade a projective reconstruction to a metric one\n"
			"required: --output=, --reconstruction=, --track= --intrinsics");

	if (!FLAGS_reconstruction.size())
		ERROR("a projective reconstruction is required (--reconstruction=)");
	if (!FLAGS_output.size())
		ERROR("output reconstruction filename  is required (--output=...)");
	if (!FLAGS_intrinsics.size())
		ERROR("camera calibration is required (--intrinsics=...)");

	TrackedSequence *ts = load_track_or_die();

	Json* js = load_json_or_die(FLAGS_reconstruction);

	ProjectiveReconstruction *prr = (ProjectiveReconstruction*)json_to_reconstruction(
			*ts, js->get("projective_reconstruction"));
	if (prr == NULL)
		ERROR("couldn't convert JSON to reconstruction (%s)",
				FLAGS_reconstruction.c_str());

	Config config_ = LoadConfig();

	fprintf(stderr, "before metric upgrade: %g\n",
			prr->rms_reprojection_error());
	if (FLAGS_trim_outliers) {
		fprintf(stderr, "trimming outliers...\n");
		prr->trim_outliers();
		fprintf(stderr, "after trimming: %g\n",
				prr->rms_reprojection_error());
		prr->bundle_adjust();
		prr->bundle_adjust();
		fprintf(stderr, "after metric upgrade and 2 bundles: %g\n",
				prr->rms_reprojection_error());
	}

	// If you don't do this, then the reconstruction could end up
	// backwards (very annoying and hard to track down!)
	if (FLAGS_force_positive) {
		fprintf(stderr, "forcing positive depth. If this freezes, it means there is a cut in your reconstruction (bad!)\n");
		force_positive_depth(prr);
	}

	Json* Kjs = load_json_or_die(FLAGS_intrinsics);
	double fx = Kjs->get("fx").as_double();
	double fy = Kjs->get("fy").as_double();
	double cx = Kjs->get("cx").as_double();
	double cy = Kjs->get("cy").as_double();
	double kk[] = {fx,0,cx, 0,fy,cy, 0,0,1};
	mat K(3,3);
	fill(K,kk);

	PARAMETER(bool, flip_metric_reconstruction, false);
	find_euclidean_upgrade_calibrated(*prr, K, flip_metric_reconstruction);

	fprintf(stderr, "converting to euclidean\n");
	EuclideanReconstruction* er = projective_to_euclidean(prr);
	er->set_config(config_);
	fprintf(stderr, "before EUCLIDEAN bundle: %g\n",
			er->rms_reprojection_error());
	er->bundle_adjust();
	fprintf(stderr, "after EUCLIDEAN bundle: %g\n",
			er->rms_reprojection_error());

	er->write_blender("blender_test.py");
	Json *ejs = er->dump_json();
	Json *out = json::NewMap();
	out->set("metric_reconstruction", ejs);
	out->set("config", config_.json());
	ToFile(*out, FLAGS_output);
}

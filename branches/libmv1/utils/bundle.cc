#include <stdio.h>
#include <vector>
#include <set>
#include "mv.h"
#include "mv_util.h"
#include "extern/google/gflags.h"
#include "json.h"
#include "json_numeric.h"
#include "reconstruction.h"

using namespace mv;

DEFINE_string(output,           "",  "Output keyframes to this file");
DEFINE_string(reconstruction,   "",  "Metric reconstruction to bundle adjust");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"bundle adjust a metric reconstruction some more\n"
			"required: --track=, --reconstruction=, --output=");

	if (!FLAGS_reconstruction.size())
		ERROR("a projective reconstruction is required (--reconstruction=)");
	if (!FLAGS_output.size())
		ERROR("output reconstruction filename  is required (--output=...)");

	TrackedSequence *ts = load_track_or_die();

	Json* js = load_json_or_die(FLAGS_reconstruction);

	Reconstruction *rec;
	string dest;

	if (js->has_key("metric_reconstruction")) {
		dest = "metric_reconstruction";
		rec = json_to_reconstruction(*ts, (*js)["metric_reconstruction"]);
	} else if (js->has_key("projective_reconstruction")) {
		dest = "projective_reconstruction";
		rec = json_to_reconstruction(*ts, (*js)["projective_reconstruction"]);
	} else {
		ERROR("Couldn't find a metric_reconstruction or a projective_reconstruction "
			   "in the JSON file (%s)", FLAGS_reconstruction.c_str());
	}

	if (rec == NULL)
		ERROR("Couldn't convert from JSON to reconstruction (%s)",
				FLAGS_reconstruction.c_str());

	double rms_error = rec->rms_reprojection_error();
	printf("BEFORE BUNDLE rms  error=%g\n",rms_error);
	printf("BEFORE BUNDLE mean error=%g\n",
			rec->mean_reprojection_error());
	rec->bundle_adjust();
	double rms_error_after = rec->rms_reprojection_error();
	printf("AFTER BUNDLE rms  error=%g\n",rms_error_after);
	printf("AFTER BUNDLE mean reprojection error=%g\n",
			rec->mean_reprojection_error());
	printf("Writing JSON to %s...", FLAGS_output.c_str());
	Json *ejs = rec->dump_json();
	Json *out = json::NewMap();
	out->set(dest, ejs);
	json::ToFile(*out, FLAGS_output);
	printf(" done.\n");
}

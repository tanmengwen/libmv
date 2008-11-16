#include <stdio.h>
#include "json_numeric.h"
#include "reconstruction.h"
#include "mv.h"
#include "mv_util.h"
#include "extern/google/gflags.h"
#include "json.h"

using namespace mv;

DEFINE_string(output,           "",  "Output keyframes to this file");
DEFINE_string(reconstruction,   "",  "Metric reconstruction to load");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
	       "export a libmv reconstruction to a blender python script\n"
		   "required: --track=, --output=, --reconstruction=\n");

	if (!FLAGS_reconstruction.size())
		ERROR("metric input reconstruction filename (--reconstruction=) is required.");
	if (!FLAGS_output.size())
		ERROR("output filename (--output=myexport.py) is required.");

	TrackedSequence *ts = load_track_or_die();

	Json* js = load_json_or_die(FLAGS_reconstruction);

	if (!js->has_key("metric_reconstruction"))
		ERROR("%s is not a euclidean reconstruction; metric upgrade first.",
				FLAGS_reconstruction.c_str());

	EuclideanReconstruction *er = (EuclideanReconstruction*)
		json_to_reconstruction(*ts, (*js)["metric_reconstruction"]);

	if (er == NULL)
		ERROR("Couldn't convert from JSON to reconstruction (%s)",
				FLAGS_reconstruction.c_str());

	printf("Writing Blender export to %s...", FLAGS_output.c_str());
	er->write_blender(FLAGS_output.c_str());
	printf(" done.\n");
}

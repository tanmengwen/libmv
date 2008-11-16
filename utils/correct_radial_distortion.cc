#include <stdio.h>
#include "mv.h"
#include "mv_util.h"
#include "json.h"
#include "radial.h"
#include "extern/google/gflags.h"
using namespace mv;

DEFINE_string(output,           "",  "Output keyframes to this file");
DEFINE_string(intrinsics,       "",  "Camera intrinsics");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"remove radial distortion from a tracked sequence\n"
			"required: --track=, --output=, --intrinsics=");

	if (!FLAGS_intrinsics.size())
		ERROR("Need to specify camera intrinsics file (--intrinsics=)");

	if (!FLAGS_output.size())
		ERROR("Missing an output file (--output=)");

	TrackedSequence *ts = load_track_or_die();

	Json *k = load_json_or_die(FLAGS_intrinsics);

	Intrinsics intrinsics(*k);

	fprintf(stderr, "Correcting distortion...");
	undistort_track(intrinsics, ts);
	fprintf(stderr, " done.\n");

	fprintf(stderr, "Writing results...");
	json::ToFile(*ts->dump_json(), FLAGS_output);
	fprintf(stderr, " done.\n");
}

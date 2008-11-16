#include <stdio.h>
#include <assert.h>
#include <vector>
#include <json.h>
#include <set>
#include <map>
#include "mv.h"
#include "mv_util.h"
#include "reconstruction.h"
#include "resection_reconstruction.h"
#include "json.h"
#include "json_numeric.h"
#include "resection.h"
#include "keyframe.h"
#include "ransac.h"
#include "extern/google/gflags.h"
using namespace mv;

DEFINE_string(output,           "",  "Output filename");
DEFINE_string(reconstruction,   "",  "Projective reconstruction to start with");
DEFINE_int32 (first_frame,       0,  "First frame to reconstruct");
DEFINE_int32 (last_frame,       -1,  "Last frame to reconstruct");
DEFINE_int32 (ransac_rounds,    -1,  "Ransac rounds for resection");
DEFINE_bool  (trim_outliers, false,  "Trim outliers before resection");
DEFINE_int32 (max_frames,       -1,  "Max frames to resection (-1 for unlimited)");
DEFINE_double(fail_threshold,  8.0,  "Threshold at which point to declare failure");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"resection some frames in a reconstruction\n"
			"required: --track=, --output=, --reconstruction=, --first_frame=, --last_frame");

	if (!FLAGS_reconstruction.size())
		ERROR("a projective reconstruction is required (--reconstruction=)");
	if (!FLAGS_output.size())
		ERROR("output reconstruction filename  is required (--output=...)");
//	if (FLAGS_first_frame == -1)
//		ERROR("first frame to resection is required (--first_frame=...)");
//	if (FLAGS_last_frame == -1)
//		ERROR("last frame to resection is required (--last_frame=...)");

	TrackedSequence *ts = load_track_or_die();

	Json* js = load_json_or_die(FLAGS_reconstruction);

	ProjectiveReconstruction *prr = (ProjectiveReconstruction*)json_to_reconstruction(
			*ts, js->get("projective_reconstruction"));

	if (prr == NULL)
		ERROR("couldn't convert JSON to reconstruction (%s)",
				FLAGS_reconstruction.c_str());

	Config config = LoadConfig();
	prr->set_config(config);

	printf("Resectioning...\n");
	resection_reconstruction(prr, FLAGS_fail_threshold, FLAGS_max_frames);

	printf("Bundle adjusting...");
	prr->bundle_adjust();
	if (FLAGS_trim_outliers) {
		prr->trim_outliers();
		prr->bundle_adjust();
	}

	force_positive_depth(prr);

	printf("Dumping results to JSON %s...\n", FLAGS_output.c_str());
	Json* js2 = prr->dump_json();
	Json* out = json::NewMap();
	out->set("config", config.json());
	out->set("projective_reconstruction", js2);
	json::ToFile(*out, FLAGS_output);
	printf("...done\n");
}

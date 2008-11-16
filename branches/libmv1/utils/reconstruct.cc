#include <stdio.h>
#include <assert.h>
#include <vector>
#include <json.h>
#include "mv.h"
#include "mv_util.h"
#include "reconstruction.h"
#include "keyframe.h"
#include "config.h"
#include "extern/google/gflags.h"

using namespace mv;

DEFINE_string(output,           "",  "Output subsets to this file");
DEFINE_string(keyframes_file,   "",  "JSON file to read keyframes from");
DEFINE_int32 (first,             0,  "First frame of the sequence for sprintf'ing into pattern");
DEFINE_int32 (last,             -1,  "Last frame");
DEFINE_int32 (ransac_rounds,   300,  "Number of ransac rounds in 3point reconstruction");
DEFINE_bool  (bundle_subsets, true,  "Bundle adjust subsets after each one is created.");
DEFINE_bool  (nview,          true,  "Use nview rather than threeview code for subsets");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"reconstruct subsets\n"
			"required: --output=, --pattern=, --start=, --end=");

	if (!FLAGS_output.size())
		ERROR("output reconstruction filename (--output=...) is required.");

	TrackedSequence *ts = load_track_or_die();

	std::vector<size_t> keyframes;
	if (FLAGS_keyframes_file.size()) {
		json::Json *kfs = load_json_or_die(FLAGS_keyframes_file);
		json::Json &js_keyframes = *kfs;
		printf("Keyframes chosen: [");
		size_t num_keyframes = js_keyframes["keyframes"].size();
		keyframes.resize(num_keyframes);
		for (size_t i=0; i<num_keyframes; i++) {
			keyframes[i] = js_keyframes["keyframes"][i].as_int();
			printf("%d, ", keyframes[i]);
		}
		printf("]\n");
		delete kfs;
	} else {
		printf("No keyframes specified, reconstrucing all frames\n");
		keyframes.resize(ts->size());
		for (size_t i=0; i<ts->size(); i++) {
			keyframes[i] = i;
		}
	}

	printf("Reconstructing 3-frame subsets (with keyframe goodness)\n");
	std::vector<Reconstruction*> prs;

	Config config = LoadConfig();
	if (FLAGS_nview) {
		reconstruct_keyframes_nview(*ts, FLAGS_ransac_rounds, keyframes, &prs, &config);
	} else {
		reconstruct_keyframes(*ts, FLAGS_ransac_rounds, keyframes, &prs, &config);
	}
	printf("Finished subsets.\n");

	if (FLAGS_bundle_subsets) {
		printf("Bundling 3-frame subsets\n");
		bundle_adjust_subsets(prs);
	} else {
		printf("Skipping bundle adjustment\n");
	}

	printf("Dumping 3-frame subsets to JSON %s...\n", FLAGS_output.c_str());
	Json* out = reconstructions_to_json(prs);
	out->set("config", config.json());
	json::ToFile(*out, FLAGS_output);
	printf("... done.\n");
}

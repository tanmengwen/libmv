#include <stdio.h>
#include <assert.h>
#include <vector>
#include <json.h>
#include "mv.h"
#include "mv_util.h"
#include "keyframe.h"
#include "config.h"
#include "extern/google/gflags.h"

using namespace mv;

DEFINE_string(output,           "",  "Output keyframes to this file");
DEFINE_int32 (first_frame,       0,  "First frame of the sequence to consider");
DEFINE_int32 (last_frame,       -1,  "Last frame of the sequence to consider");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"pick subsets of frames to reconstruct from\n"
			"required: --output=, ");

	TrackedSequence *ts = load_track_or_die();

	if (FLAGS_last_frame != -1)
		printf("Only reconstructing first %d frames\n",FLAGS_last_frame);
	else
		FLAGS_last_frame = ts->size();

	Config config = LoadConfig();

	std::vector<size_t> keyframes;
	pick_keyframes(*ts, &keyframes, &config);

	Json *out = json::NewMap();
	Json &jskeyframes = out->set_new_array("keyframes");
	for (size_t i=0; i<keyframes.size(); i++) {
		if (keyframes[i] >= (size_t)FLAGS_first_frame
				&& keyframes[i] <= (size_t)FLAGS_last_frame)
			jskeyframes.append(keyframes[i]);
	}
	out->set("config", config.json());
	json::ToFile(*out, FLAGS_output);
	delete out;
}

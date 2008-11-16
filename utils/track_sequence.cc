#include <stdio.h>
#include "mv.h"
#include "mv_util.h"
#include <json.h>
#include "config.h"
#include "extern/google/gflags.h"

using namespace mv;

DEFINE_string(pattern,          "",  "Frame pattern, e.g. ../frames-%08d.pgm");
DEFINE_string(output,           "",  "Output filename");
DEFINE_int32 (start,             0,  "First frame of the sequence for sprintf'ing into pattern");
DEFINE_int32 (end,              -1,  "Last frame");
DEFINE_int32 (num_features,    500,  "Number of features to track");
DEFINE_bool  (debug,         false,  "Output debug information");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"track salient features through an image sequence\n"
			"required: --output=, --pattern=, --start=, --end=");

	if (FLAGS_output.size() == 0)
		ERROR("need to specify output filename (--output=mytrack.ts.json)");
	if (FLAGS_pattern.size() == 0)
		ERROR("need to specify an image pattern (--pattern=frame%%08d.pgm)");
	if (FLAGS_end <= FLAGS_start)
		ERROR("end <= start; no frames to track!");

	Config config = LoadConfig();
	config.set_parameter("num_features", FLAGS_num_features);

	TrackedSequence ts;
	ts.set_config(config);

	if (FLAGS_debug)
		ts.enable_debug_output();

	printf("Tracking %d features.\n", FLAGS_num_features);
	ts.track_pgm_sequence(
			FLAGS_pattern.c_str(),
			FLAGS_start,
			FLAGS_end);

	printf("Dumping to JSON file %s\n", FLAGS_output.c_str());
	json::Json* out = ts.dump_json();
	out->set("config", config.json());
	json::ToFile(*out, FLAGS_output);
}

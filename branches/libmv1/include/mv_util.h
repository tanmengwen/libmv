#ifndef _MV_UTIL_H
#define _MV_UTIL_H

namespace mv {

#define ERROR(...) { \
	fprintf(stderr, "ERROR: " __VA_ARGS__); \
	fprintf(stderr, "\n"); \
	exit(-1); \
}

#define SET_USAGE_AND_PARSE_ARGS(x) \
	string usage(x); \
	google::SetUsageMessage(usage); \
	google::ParseCommandLineFlags(&argc, &argv, true);


Json* load_json_or_die(const string& filename);
TrackedSequence* load_track_or_die();

} // namespace mv

#endif // _MV_UTIL_H

#include <errno.h>
#include <string.h>

#include "mv.h"
#include "mv_util.h"
#include "extern/google/gflags.h"

DEFINE_string(track, "",  "Track file");

namespace mv {

Json* load_json_or_die(const string& filename)
{
	fprintf(stderr, "Loading JSON file %s...", filename.c_str());

	Json *js = json::FromFile(filename);
	if (js == NULL) {
		int old_errno = errno; // need to save errno
		fprintf(stderr, "\nERROR: couldn't read file '%s' (%s)\n",
				filename.c_str(),
				strerror(old_errno));
		exit(-1);
	}
	fprintf(stderr, " done.\n");
	return js;
}

TrackedSequence* load_track_or_die()
{

	if (!FLAGS_track.size())
		ERROR("track filename (--track=) is required.");

	TrackedSequence *ts = json_to_tracked_sequence(
			load_json_or_die(FLAGS_track));
	return ts;
}

} // namespace mv

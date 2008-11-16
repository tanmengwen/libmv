#include <stdio.h>
#include "mv.h"
#include "mv_util.h"
#include "json.h"
#include "json_numeric.h"
#include "reconstruction.h"
#include "ground_truth.h"
#include "extern/google/gflags.h"
using namespace mv;
using namespace json;
char* OPTIONS = "ht:m:g:n:N:";

DEFINE_string(output,           "",  "Output filename (json)");
DEFINE_string(reconstruction, "",  "Metric reconstruction to evaluate (optional)");
DEFINE_string(ground_truth,     "",  "Pattern for ground truth exr data");
DEFINE_int32 (first,             1,  "First frame of the sequence for sprintf'ing into pattern");
DEFINE_int32 (last,             -1,  "Last frame");

void evaluate_metric_reconstruction(
		TrackedSequence *ts,
		EuclideanReconstruction *er,
		GroundTruth &truth,
		Json *out);

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"evaluate tracks and reconstructions\n"
			"required: --track=, --ground_truth=, --first=, --last=");

	if (!FLAGS_output.size())
		ERROR("need an output file (--output=...)");
	if (!FLAGS_ground_truth.size())
		ERROR("need exr ground truth data (--ground_truth=...)");
	if (FLAGS_last == -1)
		ERROR("must specify last frame (--last=...)");

	TrackedSequence *ts = load_track_or_die();
	
	GroundTruthEXRLoader loader;
	loader.load_ground_truth(FLAGS_ground_truth.c_str(), -1, -1); // first/last are not used atm
	GroundTruth truth;
	truth.set_loader(&loader);

	Json *out = NewMap();

	EuclideanReconstruction *mr = NULL;
	if (FLAGS_reconstruction.size()) {
		fprintf(stderr, "Loading reconstruction %s...\n", FLAGS_reconstruction.c_str());
		Json *js = load_json_or_die(FLAGS_reconstruction);
		mr = (EuclideanReconstruction*) json_to_reconstruction(*ts, js->get("metric_reconstruction"));
		delete js;
		if (mr == NULL) {
			fprintf(stderr, "ERROR: couldn't open reconstruction file %s\n",
					FLAGS_reconstruction.c_str());
			exit(-1);
		}
		evaluate_metric_reconstruction(ts, mr, truth, out);
	} else {
		// Push the track directly into the reconstruction
		for (size_t i=0; i<ts->size(); i++) {
			int orig_frame_number = ts->at(i)->get_orig_ind();
			for (size_t j=0; j<ts->at(i)->size(); j++) {
				PointFeature* pf = (PointFeature*) ts->at(i)->at(j);
				truth.add_point(
						orig_frame_number,
						pf->get_track_number(),
						pf->x(), pf->y());
			}
		}
		Json *tracks = truth.tracks_to_json();
		out->set("tracks", tracks);
	}
	ToFile(*out, FLAGS_output);
}

void evaluate_metric_reconstruction(
		TrackedSequence *ts,
		EuclideanReconstruction *mr,
		GroundTruth &truth,
		Json *out) {

	mr->number_items();

	// push structure and measurements into ground truth (and force loads)
	// Structure
	for (size_t i=0; i<mr->structure.size(); i++) {
		EuclideanPointStructure* ps = (EuclideanPointStructure*)mr->structure[i];
		assert(ps->get_number() == i);
		truth.set_estimated_track_position(ps->get_number(),
				ps->X[0], ps->X[1], ps->X[2]);
	}

	if (FLAGS_last == -1)
		FLAGS_last = mr->cameras.size()-1;

	int current_frame = FLAGS_first;

	// dumbass n^2 search to force sequential frame loading
	for (size_t i=0; i<mr->cameras.size(); i++) {
		int ts_frame_number = mr->ecam(i)->get_frame_number();
		int orig_frame_number = ts->at(ts_frame_number)->get_orig_ind();
		MeasurementVisitor mv(mr);
		while (mv.next()) {
			int f = mv.camera()->get_frame_number();
			int ofn = ts->at(f)->get_orig_ind();
			if (ofn != orig_frame_number)
				continue;
			PointFeature* pf = (PointFeature*) mv.measurement();
			// current_frame is FILE-referenced; aka relates to original files
			// we tracked from, and may not have started from 0
			truth.add_point(
					orig_frame_number,
					mv.structure()->get_number(),
					pf->x(), pf->y());
		}
		fprintf(stderr, "frame_ind=%d; cam_ind=%d, current_frame=%d, orig_ind=%d, frame_filename=%s\n",
				ts_frame_number, i, current_frame, orig_frame_number,
				mr->ecam(i)->get_frame()->get_filename().c_str());

		current_frame++;
		if (current_frame == FLAGS_last)
			break;
	}

	// FIXME make these exported names more meaningful
	double error_thresh = truth.align();
	mat all_residuals = truth.residuals();
//	truth.export_truth_blender(reconstruction_fn);
	out->set("all_residuals", mat2json(all_residuals));
	Json *tracks = truth.tracks_to_json();
	out->set("tracks", tracks);
	Json *est = truth.estimated_to_json();
	out->set("est", est);
	out->set("error_thresh", error_thresh);
}

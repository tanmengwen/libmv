#include <stdio.h>
#include <vector>
#include <json.h>
#include "mv.h"
#include "mv_util.h"
#include "reconstruction.h"
#include "extern/google/gflags.h"
using namespace mv;

DEFINE_string(output,           "",  "Output merged projective reconstruction to this file");
DEFINE_string(subsets,          "",  "Subsets to merge");
DEFINE_int32 (first_subset,      0,  "First subset of the sequence to use (inclusive)");
DEFINE_int32 (last_subset,      -1,  "Last subset to use (inclusive)");
//DEFINE_int32 (ransac_rounds,   100,  "Number of ransac rounds in 3point reconstruction");
DEFINE_bool  (bundle_subsets, true,  "Bundle adjust subsets after each one is created.");

int main(int argc, char* argv[])
{
	SET_USAGE_AND_PARSE_ARGS(
			"merge subsets\n"
			"required: --output=, --subsets=, --track=");

	if (!FLAGS_subsets.size())
		ERROR("subsets are required (--subsets=)");
	if (!FLAGS_output.size())
		ERROR("output reconstruction filename (--output=...) is required.");

	TrackedSequence *ts = load_track_or_die();

	printf("Loading subsets...\n");
	std::vector<Reconstruction*> prs;
	Json* js = load_json_or_die(FLAGS_subsets);
	json_to_reconstructions(*ts, *js, &prs);
	delete js;
	printf("Found %d subsets.\n", prs.size());

	if (FLAGS_first_subset) {
		printf("Starting reconstruction at subset %d.\n", FLAGS_first_subset);
		prs.erase(prs.begin(),prs.begin()+FLAGS_first_subset-1);
	}

	if (FLAGS_last_subset != -1 && FLAGS_last_subset < (int)prs.size()) {
		printf("Ending reconstruction at subset %d.\n", FLAGS_last_subset);
		prs.erase(prs.begin()+FLAGS_last_subset+1-FLAGS_first_subset,prs.end());
	}

	Config config = LoadConfig();
	for (size_t i=0; i<prs.size(); i++) {
		ProjectiveReconstruction *hack = (ProjectiveReconstruction *)prs[i];
		hack->set_config(config);
	}

	printf("Hierarchicaly merging subsets\n");
	ProjectiveReconstruction *prr = merge_hierarchical(*ts, prs);

	printf("Finished merging. Saving...\n");
	js = prr->dump_json();
	json::Json *out = json::NewMap();
	out->set("projective_reconstruction", js);
	out->set("config", config.json());
	json::ToFile(*out, FLAGS_output);
	printf("Saved to %s\n", FLAGS_output.c_str());
	delete js;
}

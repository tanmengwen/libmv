#include "testmv.h"
#include "ransac.h"
#include "test_tracked_sequence_data.h"

#if 1
TEST(reconstruction_saving_loading) {
	char reconstruction_fn[] = "TEST_reconstruction_dump.pr";
	TrackedSequence ts;
	make_track_data(ts);
	ProjectiveReconstruction *pr = reconstruct_hierarchical(ts,200,4);

	dump_reconstruction_to_file(*pr, reconstruction_fn);
	ProjectiveReconstruction *prr = read_reconstruction_from_file(ts, reconstruction_fn);

	reconstructions_match_exactly(pr, prr);
}
#endif

#if 1
TEST(many_reconstruction_saving_loading) {
	char many_fn[] = "TEST_many_reconstructions.prs";

	TrackedSequence ts;
	make_track_data(ts);
	std::vector<ProjectiveReconstruction*> prs;
	reconstruct_sequential_subsets(ts, 100, 3, &prs);
	dump_many_reconstructions(many_fn, prs);

	std::vector<ProjectiveReconstruction*> read_prs;
	read_many_reconstructions(ts, many_fn, &read_prs);
	Equals(prs.size(), read_prs.size());

	for (size_t i=0; i<prs.size(); i++)
		reconstructions_match_exactly(prs[i], read_prs[i]);
}
#endif

TEST(copy_projective_reconstruction) {
	char track_fn[] = "test_data/blender1_5frame.ts";
	TrackedSequence *tss = read_from_file(track_fn);

	std::vector<ProjectiveReconstruction*> prs;
	reconstruct_sequential_subsets(*tss, 10, 4, &prs);

	for (size_t i=0; i<prs.size(); i++) {
		printf("checking copy of reconstruction %d\n", i);
		ProjectiveReconstruction* pr_copy = (ProjectiveReconstruction*) prs[i]->copy();
		reconstructions_match_exactly(prs[i], pr_copy);
		// FIXME leaky!
	}
}

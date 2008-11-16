#include "testmv.h"

TEST(tracks_saving_loading) {
	char format[] = "/home/keir/thesis/movies/blender_test1/%04d.pgm";
	char track_fn[] = "test_track_dump.ts";
	TrackedSequence ts;
	ts.enable_debug_output();
//	ts.track_pgm_sequence(format, 28, 64, 500);
	ts.track_pgm_sequence(format, 28, 32, 1000);

	dump_to_file(ts, track_fn);
	TrackedSequence *tss = read_from_file(track_fn);

	Equals (ts.size(), tss->size());
	Equals (ts.tracks.size(), tss->tracks.size());
	for (size_t i=0; i<ts.size(); i++) {
		Equals(ts[i]->size(), (*tss)[i]->size());
		Frame* fr1 = ts[i];
		Frame* fr2 = (*tss)[i];
		for (size_t j=0; j<fr1->size(); j++) {
			PointFeature *pf1 = (PointFeature*) (*fr1)[j];
			PointFeature *pf2 = (PointFeature*) (*fr2)[j];
			Equals(pf1->x(), pf2->x());
			Equals(pf1->y(), pf2->y());
			Equals(pf1->get_frame()->get_frame_number(),
			       pf2->get_frame()->get_frame_number());
			Equals(pf1->get_frame_number(),
			       pf2->get_frame_number());
			Equals(pf1->get_track()->get_number(),
			       pf2->get_track()->get_number());
		}
	}
	for (size_t i=0; i<ts.tracks.size(); i++) {
		Equals(ts.tracks[i]->size(), tss->tracks[i]->size());
		Track* tr1 = ts.tracks[i];
		Track* tr2 = tss->tracks[i];
		for (size_t j=0; j<tr1->size(); j++) {
			Equals((*tr1)[j]->get_frame_number(),
			       (*tr2)[j]->get_frame_number());
			Equals((*tr1)[j]->get_number(),
			       (*tr2)[j]->get_number());
		}
	}
}
#endif

void reconstructions_match_exactly(
		ProjectiveReconstruction* pr, 
		ProjectiveReconstruction *prr)
{
	Equals(pr->cameras.size(),      prr->cameras.size());
	Equals(pr->structure.size(),    prr->structure.size());
	Equals(pr->measurements.size(), prr->measurements.size());
	Equals(pr->cam_sees.size(),     prr->cam_sees.size());
	Equals(pr->visible_in.size(),   prr->visible_in.size());

	for (size_t k=0; k<pr->cameras.size(); k++) {
		ProjectiveCamera* cam1 =  pr->pcam(k);
		ProjectiveCamera* cam2 = prr->pcam(k);
//		printf("k=%d\n",k);
		for (size_t i=0; i<3; i++) 
			for (size_t j=0; j<4; j++) 
				Equals((*cam1)(i,j), (*cam2)(i,j));
		Equals(cam1->number, cam2->number);
		assert(cam1->get_frame_number() ==
		       cam2->get_frame_number());
		Equals(cam1->get_frame_number(),
		       cam2->get_frame_number());
	}

	// Make sure the measurements match the tracks
	// FIXME this will never succeed for now, because we are not pruning
	// tracks which are inliers in one subset but outliers in another.
	/*
	size_t track_measurement_total1 = 0;
	size_t track_measurement_total2 = 0;
	for (size_t i=0; i<pr->structure.size(); i++) {
		Track* track1 =  pr->structure[i]->get_track();
		Track* track2 = prr->structure[i]->get_track();
		track_measurement_total1 += track1->size();
		track_measurement_total2 += track2->size();
	}
	Equals(track_measurement_total1,  pr->measurements.size());
	Equals(track_measurement_total2, prr->measurements.size());
	*/

	MeasurementVisitor mv1(pr);
	MeasurementVisitor mv2(prr);

	while (mv1.next()) {
		Equals(mv2.next(), 1);

		PointStructure* ss1 = (PointStructure*) mv1.structure();
		ProjectiveCamera* cam1 = (ProjectiveCamera*) mv1.camera();
		PointFeature* f1 = (PointFeature*) mv1.measurement();

		PointStructure* ss2 = (PointStructure*) mv2.structure();
		ProjectiveCamera* cam2 = (ProjectiveCamera*) mv2.camera();
		PointFeature* f2 = (PointFeature*) mv2.measurement();

		for (size_t i=0; i<3; i++) 
			for (size_t j=0; j<4; j++) 
				Equals((*cam1)(i,j), (*cam2)(i,j));
		for (size_t j=0; j<4; j++) 
			Equals((*ss1)[j], (*ss2)[j]);

		Equals(cam1->number, cam2->number);
		assert(cam1->get_frame_number() ==
		       cam2->get_frame_number());
		Equals(cam1->get_frame_number(),
		       cam2->get_frame_number());
		Equals(ss1->number, ss2->number);
		Equals(f1->x(), f2->x());
		Equals(f1->y(), f2->y());
		Equals(f1->get_number(), f2->get_number());
		Equals(f1->get_frame_number(), f2->get_frame_number());
	}
	Equals(mv2.next(), 0);
}

#if 1
TEST(reconstruction_saving_loading) {
//	char track_fn[] = "test_data/test_track_5_frames.ts";
	char reconstruction_fn[] = "test_reconstruction_dump.pr";
	char track_fn[] = "test_track_dump.ts";
	TrackedSequence *tss = read_from_file(track_fn);

	ProjectiveReconstruction *pr = reconstruct_hierarchical(*tss,200,-1);
	pr->print_cameras();

	dump_reconstruction_to_file(*pr, reconstruction_fn);
	ProjectiveReconstruction *prr = read_reconstruction_from_file(*tss, reconstruction_fn);

	reconstructions_match_exactly(pr, prr);

	// FIXME check BOTH cam_sees and visible_in rather than just the one
	// that measurementvisitor uses
}
#endif

TEST(many_reconstruction_saving_loading) {
//	char track_fn[] = "test_track_dump.ts";
	char track_fn[] = "test_data/blender1_5frame.ts";
	char many_fn[] = "test_many_reconstructions.prs";
	TrackedSequence *tss = read_from_file(track_fn);
//	tss->number_items();

	std::vector<ProjectiveReconstruction*> prs;
	reconstruct_sequential_subsets(*tss, 10, 4, &prs);
	// used for tracking down a bug...
	/*
	for (size_t j=0; j<2; j++) {
		for (size_t i=0; i<prs[j]->cameras.size(); i++) {
			printf("pre-dump cam %d frame=%p frame_nr=%d\n",i,
					prs[j]->pcam(i)->frame,
					prs[j]->pcam(i)->frame->get_frame_number()
					);
		}
	}
	*/
	dump_many_reconstructions(many_fn, prs);

	std::vector<ProjectiveReconstruction*> read_prs;
	read_many_reconstructions(*tss, many_fn, &read_prs);
	Equals(prs.size(), read_prs.size());

	for (size_t i=0; i<prs.size(); i++) {
//		printf("checking reconstruction %d\n", i);
		reconstructions_match_exactly(prs[i], read_prs[i]);
	}
}

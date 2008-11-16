#include <testmv.h>
#include "camera.h"
#include "reconstruction.h"

TEST(empty_projective_reconstruction) {
	ProjectiveReconstruction rec;
	Json &m = *rec.dump_json();
	Equals(m["type"].as_string(), "projective reconstruction");
	Equals(m["cameras"].size(), 0u);
	Equals(m["structure"].size(), 0u);
	Equals(m["measurements"].size(), 0u);
}

TEST(cameras_with_no_visibility) {
	ProjectiveReconstruction rec;
	ProjectiveCamera cam1;
	rec.push_camera(&cam1);
	PointStructure s1;
	PointStructure s2;
	rec.push_structure(&s1);
	rec.push_structure(&s2);
	Json &m = *rec.dump_json();
	Equals(m["type"].as_string(), "projective reconstruction");
	Equals(m["cameras"].size(), 1u);
	Equals(m["structure"].size(), 2u);
	Equals(m["measurements"].size(), 0u);
}

//#include <iostream>
TEST(cameras_with_no_visibility) {
	ProjectiveReconstruction rec;
	ProjectiveCamera cam1;
	rec.push_camera(&cam1);
	PointStructure s1;
	PointStructure s2;
	rec.push_structure(&s1);
	rec.push_structure(&s2);
	rec.push_measurement(&cam1, &s2, NULL);
	Json &m = *rec.dump_json();
	Equals(m["type"].as_string(), "projective reconstruction");
	Equals(m["cameras"].size(), 1u);
	Equals(m["structure"].size(), 2u);
	Equals(m["measurements"].size(), 0u);
}

void reconstructions_match_exactly(
		Reconstruction* pr1, 
		Reconstruction *pr2)
{
	Equals(pr1->cameras.size(),      pr2->cameras.size());
	Equals(pr1->structure.size(),    pr2->structure.size());
	Equals(pr1->measurements.size(), pr2->measurements.size());
	Equals(pr1->cam_sees.size(),     pr2->cam_sees.size());
	Equals(pr1->visible_in.size(),   pr2->visible_in.size());

	for (size_t i=0; i<pr1->cameras.size(); i++) {
		Equals(pr1->cameras[i]->equals(*pr2->cameras[i]),
				true);
	}

	MeasurementVisitor mv1(pr1);
	MeasurementVisitor mv2(pr2);

	while (mv1.next()) {
		Equals(mv2.next(), 1);

		Structure* ss1 =   mv1.structure();
		Camera* cam1 =     mv1.camera();
		PointFeature* f1 = (PointFeature*)mv1.measurement();

		Structure* ss2 =   mv2.structure();
		Camera* cam2 =     mv2.camera();
		PointFeature* f2 = (PointFeature*)mv2.measurement();

		Equals(cam1->equals(*cam2), true);
		Equals(ss1->equals(*ss2), true);

		Equals(f1->x(), f2->x());
		Equals(f1->y(), f2->y());
		Equals(f1->get_number(), f2->get_number());
		Equals(f1->get_frame_number(), f2->get_frame_number());
	}
	Equals(mv2.next(), 0);
}

TEST(projective_reconstruction_to_json_and_back_again) {
	ProjectiveReconstruction rec;
	ProjectiveCamera cam1;
	rec.push_camera(&cam1);
	PointStructure s1;
	PointStructure s2;
	rec.push_structure(&s1);
	rec.push_structure(&s2);
	TrackedSequence ts;
	ts.add_point_feature(0,0,43,44);
	ts.add_point_feature(0,1,43,44);
	cam1.set_frame(ts[0]);
	s1.set_track(ts.tracks[0]);
	s2.set_track(ts.tracks[1]);
	rec.push_measurement(&cam1, &s2, ts[0]->at(0));

	Json &m = *rec.dump_json();
	Reconstruction *reloaded = json_to_reconstruction(ts, m);

	reconstructions_match_exactly(&rec, reloaded);
}

TEST(euclidean_reconstruction_to_json_and_back_again) {
	EuclideanReconstruction rec;
	EuclideanCamera cam1;
	rec.push_camera(&cam1);
	EuclideanPointStructure s1;
	EuclideanPointStructure s2;
	rec.push_structure(&s1);
	rec.push_structure(&s2);
	TrackedSequence ts;
	ts.add_point_feature(0,0,43,44);
	ts.add_point_feature(0,1,43,44);
	cam1.set_frame(ts[0]);
	s1.set_track(ts.tracks[0]);
	s2.set_track(ts.tracks[1]);
	rec.push_measurement(&cam1, &s2, ts[0]->at(0));

	Json &m = *rec.dump_json();
	Reconstruction *reloaded = json_to_reconstruction(ts, m);

	reconstructions_match_exactly(&rec, reloaded);
}

// FIXME: Decide if these tests should be kept. Underlying code is not used
// anymore, because we've switched to JSON.
#if 0
#include "test_tracked_sequence_data.h"
TEST(many_reconstruction_saving_loading) {
	char many_fn[] = "TEST_many_reconstructions.prs";

	TrackedSequence ts;
	make_track_data(ts);
	std::vector<Reconstruction*> prs;
	reconstruct_sequential_subsets(ts, 100, 4, &prs);
//	std::cout << prs[0]->dump_json()->to_string() << std::endl;
//	std::cout << prs[1]->dump_json()->to_string() << std::endl;
//	std::cout << prs.size() << std::endl;
	dump_many_reconstructions(many_fn, prs);

	std::vector<ProjectiveReconstruction*> read_prs;
	read_many_reconstructions(ts, many_fn, &read_prs);
	Equals(prs.size(), read_prs.size());

	for (size_t i=0; i<prs.size(); i++)
		reconstructions_match_exactly(prs[i], read_prs[i]);
}

TEST(many_reconstruction_saving_loading) {
	char many_fn[] = "TEST_many_reconstructions.prs";

	TrackedSequence ts;
	make_track_data(ts);
	std::vector<Reconstruction*> prs;
	reconstruct_sequential_subsets(ts, 100, 4, &prs);
//	std::cout << prs[0]->dump_json()->to_string() << std::endl;
//	std::cout << prs[1]->dump_json()->to_string() << std::endl;
//	std::cout << prs.size() << std::endl;
	dump_many_reconstructions(many_fn, prs);

	std::vector<ProjectiveReconstruction*> read_prs;
	read_many_reconstructions(ts, many_fn, &read_prs);
	Equals(prs.size(), read_prs.size());

	for (size_t i=0; i<prs.size(); i++)
		reconstructions_match_exactly(prs[i], read_prs[i]);
}
#endif

#include <vector>
#include <testmv.h>
#include "test_tracked_sequence_data.h"

TEST(ts_add_features_easily_one) {
	TrackedSequence ts;
	ts.add_point_feature(0,0, 10.0,11.0);
	Equals(ts.size(), 1u);
	Equals(ts.tracks.size(), 1u);
}

TEST(ts_add_features_easily_four_tracks) {
	TrackedSequence ts;
	ts.add_point_feature(0,3, 10.0,11.0);
	Equals(ts.size(), 1u);
	Equals(ts.tracks.size(), 4u);
}

TEST(ts_add_features_easily_five_tracks_and_frames) {
	TrackedSequence ts;
	ts.add_point_feature(0,4, 1.0,2.0);
	ts.add_point_feature(4,2, 2.0,4.0);
	Equals(ts.size(), 5u);
	Equals(ts.tracks.size(), 5u);
	Equals(((PointFeature*)(ts[0]->at(0)))->x(), 1.0);
	Equals(((PointFeature*)(ts[4]->at(0)))->y(), 4.0);
}

TEST(frame_collection_starts_with_null_frames) {
	FrameCollection fc(10);
	Null(fc[0]);
}

TEST(ts_dump_cpp_ascii_trivial) {
	TrackedSequence ts;
	string result = "TrackedSequence ts;\n";
	string dump = ts.dump_cpp_ascii();
	Equals(dump, result);
}

TEST(ts_dump_cpp_ascii_one_point) {
	TrackedSequence ts;
	ts.add_point_feature(0,0, 1.1,2.2);
	string result = 
		"TrackedSequence ts;\n"
		"ts.add_point_feature(0,0, 1.1,2.2);\n";
	string dump = ts.dump_cpp_ascii();
	Equals(dump, result);
}

TEST(ts_dump_cpp_ascii_two_points) {
	TrackedSequence ts;
	ts.add_point_feature(0,4, 1.5,2.2);
	ts.add_point_feature(4,2, 2.5,4.4);
	string result = 
		"TrackedSequence ts;\n"
		"ts.add_point_feature(0,4, 1.5,2.2);\n"
		"ts.add_point_feature(4,2, 2.5,4.4);\n";
	string dump = ts.dump_cpp_ascii();
	Equals(dump, result);
}

TEST(ts_dump_cpp_ascii_one_point_with_filename) {
	TrackedSequence ts;
	ts.add_point_feature(0,0, 1.1,2.2);
	ts[0]->set_filename("foo04.pgm");
	string result = 
		"TrackedSequence ts;\n"
		"ts.add_point_feature(0,0, 1.1,2.2);\n"
		"ts[0]->set_filename(\"foo04.pgm\");\n";
	string dump = ts.dump_cpp_ascii();
	Equals(dump, result);
}

void tracks_match(TrackedSequence *ts1, TrackedSequence *ts2) {
	Equals (ts1->size(), ts2->size());
	Equals (ts1->tracks.size(), ts2->tracks.size());
	Equals (string(ts1->get_format()), string(ts2->get_format()));
	for (size_t i=0; i<ts1->size(); i++) {
		Equals(ts1->at(i)->size(), (*ts2)[i]->size());
		Frame* fr1 = ts1->at(i);
		Frame* fr2 = ts2->at(i);
		Equals(fr1->get_filename(), fr2->get_filename());
		Equals(fr1->get_orig_ind(), fr2->get_orig_ind());
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
	for (size_t i=0; i<ts1->tracks.size(); i++) {
		Equals(ts1->tracks[i]->size(), ts2->tracks[i]->size());
		Track* tr1 = ts1->tracks[i];
		Track* tr2 = ts2->tracks[i];
		for (size_t j=0; j<tr1->size(); j++) {
			Equals((*tr1)[j]->get_frame_number(),
			       (*tr2)[j]->get_frame_number());
			Equals((*tr1)[j]->get_number(),
			       (*tr2)[j]->get_number());
		}
	}
}

//TEST(tracks_saving_loading_json) {
//}

TEST(tracks_saving_loading_old_binary) {
	char track_fn[] = "TEST_track_dump.ts";
	TrackedSequence ts;
	make_track_data(ts);

	dump_to_file(ts, track_fn);
	TrackedSequence *tss = read_from_file(track_fn);

	tracks_match(&ts, tss);
}

TEST(tracks_saving_loading_json) {
	TrackedSequence ts;
	make_track_data(ts);
	
	Json* js = ts.dump_json();
	TrackedSequence *tss = json_to_tracked_sequence(js);
	delete js;

	tracks_match(&ts, tss);
}

TEST(tracks_saving_loading_json_orig_ind_diffs) {
	TrackedSequence ts;
	make_track_data(ts);
	ts[0]->set_orig_ind(10);
	Json* js = ts.dump_json();
	TrackedSequence *tss = json_to_tracked_sequence(js);
	delete js;
	tracks_match(&ts, tss);
}

TEST(tracks_saving_loading_json_format_diffs) {
	TrackedSequence ts;
	make_track_data(ts);
	ts.set_format("not empty");
	Json* js = ts.dump_json();
	TrackedSequence *tss = json_to_tracked_sequence(js);
	delete js;
	tracks_match(&ts, tss);
}

TEST(tracks_saving_loading_json_with_parse) {
	TrackedSequence ts;
	make_track_data(ts);

	Json* js = ts.dump_json();
	string dump = js->to_string();
	delete js;
	Json* js_parsed = Parse(dump);
	TrackedSequence *tss = json_to_tracked_sequence(js_parsed);
	delete js_parsed;

	tracks_match(&ts, tss);
}

TEST(tracks_dump_json_simple) {
	TrackedSequence ts;
	make_track_data(ts);
	Json *jts = ts.dump_json();
	Equals(jts->get("num_frames").as_double(), (double)ts.size());
	Equals(jts->get("num_tracks").as_double(), (double)ts.tracks.size());
}

#if 0
// FIXME i decided not to bother with this; though perhaps it should be done
// someday!
TEST(trim_short_tracks) {
	TrackedSequence ts;
	make_track_data(ts);

	int min_track_length = 4;
	TrackedSequence *ts2 = trim_tracks_shorter_than(min_track_length);

	// now make sure each track is shorter need this loop because there may be
	// hanging track refs in the features
	foreach (Frame* fr, *ts2) {
		foreach (Feature* ft, *fr) {
			Track* t = ft->get_track();
			GreaterThanEq(t->size(), min_track_length)
		}
	}
	for (size_t i=0; i<ts2->tracks.size(); i++) {
		GreaterThanEq(ts2->tracks->at(i)->size(), min_track_length)
	}
}
#endif

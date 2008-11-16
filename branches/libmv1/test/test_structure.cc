#include <testmv.h>
#include "structure.h"
#include "test_tracked_sequence_data.h"

TEST(projective_point_structure_serialization) {
	PointStructure X;
	X[0] = 4;
	X[1] = 3;
	X[2] = 2;
	X[3] = 1;
	Track track;
	track.set_number(10);
	X.set_track(&track);
	Json* js = X.dump_json();
	Equals(js->to_string(), 
			"{\"X\":[4,3,2,1],\"track\":10,"
			"\"type\":\"projective point structure\"}");
	delete js;
}

TEST(projective_point_structure_unserialization) {
	Json* js = json::Parse(
			"{\"X\":[4,3,2,1],\"track\":2,"
			"\"type\":\"projective point structure\"}");
	TrackedSequence ts;
	make_track_data(ts);
	PointStructure *XX;
	XX = json_to_point_structure(ts, *js);
	PointStructure &X = *XX;
	Equals(X[0], 4);
	Equals(X[1], 3);
	Equals(X[2], 2);
	Equals(X[3], 1);
	Equals(X.get_track(), ts.tracks[2]);
	delete js;
}

TEST(euclidean_structure_serialization) {
	EuclideanPointStructure X;
	X.X[0] = 4;
	X.X[1] = 3;
	X.X[2] = 2;
	Track track;
	track.set_number(10);
	X.set_track(&track);
	Json* js = X.dump_json();
	Equals(js->to_string(), 
			"{\"X\":[4,3,2],\"track\":10,"
			"\"type\":\"euclidean point structure\"}");
	delete js;
}

TEST(euclidean_point_structure_unserialization) {
	Json* js = json::Parse(
			"{\"X\":[4,3,2],\"track\":2,"
			"\"type\":\"euclidean point structure\"}");
	TrackedSequence ts;
	make_track_data(ts);
	EuclideanPointStructure *XX;
	XX = json_to_euclidean_point_structure(ts, *js);
	EuclideanPointStructure &X = *XX;
	Equals(X.X[0], 4);
	Equals(X.X[1], 3);
	Equals(X.X[2], 2);
	Equals(X.get_track(), ts.tracks[2]);
	delete js;
}

TEST(structure_equals_double_dispatch) {
	PointStructure p1, p2;
	EuclideanPointStructure e1, e2;
	Equals(p1.equals(p2), true);
	p1[0] = 4.0;
	Equals(p1.equals(p2), false);
	Equals(e1.equals(p2), false);
	Equals(p1.equals(e2), false);
	Equals(e1.equals(e2), true);
	e1.X[0] = 4.0;
	Equals(e1.equals(e2), false);
	e2.X[0] = 4.0;
	Equals(e1.equals(e2), true);
}


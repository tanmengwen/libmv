#include <testmv.h>
#include <json.h>
using namespace json;

TEST(pointfeature_simple_operations) {
	PointFeature x(1,2);
	Equals(x[0], 1.0);
	Equals(x[1], 2.0);
	x[1] = 3.0;
	Equals(x[0], 1.0);
	Equals(x[1], 3.0);
	Equals(x.x(), 1.0);
	Equals(x.y(), 3.0);
	x *= 2;
	Equals(x.x(), 2.0);
	Equals(x.y(), 6.0);
}

TEST(pointfeature_no_frame_is_null) {
	PointFeature x(1,2);
	Null(x.get_frame());
}

TEST(pointfeature_has_frame) {
	Frame f(3);
	PointFeature x(1,2);
	x.set_frame(&f);
	Equals(x.get_frame(), &f);
}

TEST(feature_dump_json_no_track) {
	Feature x;
	Json &feat = *x.dump_json();
	Equals(feat["type"].as_string(), "base feature");
	Equals(feat.has_key("track"), false);
}

TEST(feature_dump_json_has_track_number) {
	Feature x;
	Track track;
	track.set_number(5);
	x.set_track(&track);
	Json &feat = *x.dump_json();
	Equals(feat["type"].as_string(), "base feature");
	Equals(feat["track"].as_double(), 5);
}

TEST(pointfeature_dump_json) {
	PointFeature x(1,2);
	Json &feat = *x.dump_json();
	Equals(feat["type"].as_string(), "point");
	Equals(feat.has_key("track"), false);
	Equals(feat["coords"].to_string(), "[1,2]");
}

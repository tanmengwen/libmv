#include <json.h>
#include "mv.h"
#include "linalg.h"
#include "json_numeric.h"
using namespace json;

namespace mv {

Json* EuclideanPointStructure::dump_json() {
	Json *js = NewMap();
	js->set("type", "euclidean point structure");
	js->set("X", vec2json(X));
	if (get_track())
		js->set("track", get_track_number());
	return js;
}

Json* PointStructure::dump_json() {
	Json *js = NewMap();
	js->set("type", "projective point structure");
	js->set("X", vec2json(*this));
	if (get_track())
		js->set("track", get_track_number());
	return js;
}

PointStructure* json_to_point_structure(TrackedSequence &ts, const Json &js) {
	PointStructure* X = new PointStructure;
	const Json &m = js;
	assert(m["type"].as_string() == "projective point structure");
	json2vec(&m["X"], X);
	int track_num = m["track"].as_int();
	X->set_track(ts.tracks[track_num]);
	return X;
}

EuclideanPointStructure* json_to_euclidean_point_structure(TrackedSequence &ts, const Json &js) {
	EuclideanPointStructure* X = new EuclideanPointStructure;
	assert(js["type"].as_string() == "euclidean point structure");
	json2vec(&js["X"], &X->X);
	int track_num = js["track"].as_int();
	X->set_track(ts.tracks[track_num]);
	return X;
}

// double dispatch structure comparisons
bool Structure::base_equals(const Structure&s) const {
	if (number == s.number && track == s.track)
		return true;
	return false;
}
bool Structure::equals(const PointStructure& other) const {
	(void) other;
	return false;
}
bool Structure::equals(const EuclideanPointStructure& other) const {
	(void) other;
	return false;
}

bool PointStructure::equals(const Structure &s) const {
	return s.equals(*this);
}
bool PointStructure::equals(const PointStructure &s) const {
	if (!Structure::base_equals(s))
		return false;
	for (int i=0; i<4; i++)
		if ((*this)[i] != s[i])
			return false;
	return true;
}
bool PointStructure::equals(const EuclideanPointStructure &s) const {
	(void) s;
	return false;
}

bool EuclideanPointStructure::equals(const Structure &s) const {
	return s.equals(*this);
}
bool EuclideanPointStructure::equals(const PointStructure &s) const {
	(void) s;
	return false;
}
bool EuclideanPointStructure::equals(const EuclideanPointStructure &s) const {
	if (!Structure::base_equals(s))
		return false;
	for (int i=0; i<3; i++)
		if (this->X[i] != s.X[i])
			return false;
	return true;
}
} // namespace mv

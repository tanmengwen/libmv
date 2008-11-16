#include <json.h>
using namespace json;

#include "mv.h"
#include "json_numeric.h"

namespace mv {

Feature::Feature() {
	frame = NULL;
	track = NULL;
}
Frame *Feature::get_frame() {
	return frame;
}
void Feature::set_frame(Frame* frame) {
	this->frame = frame;
}
Track *Feature::get_track() {
	return track;
}
void Feature::set_track(Track* parent) {
	track = parent;
}
int Feature::get_frame_number() {
	return frame_number;
}
void Feature::set_frame_number(int n) {
	this->frame_number = n;
}
size_t Feature::get_number() {
	return number;
}
void Feature::set_number(size_t n) {
	number = n;
}

string Feature::dump_cpp_ascii() {
	return string("");
};
Feature::~Feature() {};

int Feature::get_track_number() {
	return track->get_number();
}

Json* Feature::dump_json() {
	Json *js = NewMap();
	js->set("type", "base feature");
	if (track)
		js->set("track", get_track_number());//track->get_number());
	return js;
}

Json* PointFeature::dump_json() {
	Json *js = Feature::dump_json();
	js->set("type", "point");
	Json &arr = js->set_new_array("coords");
	arr.append(x());
	arr.append(y());
	js->set("covariance", mat2json(sigma));
	return js;
}

} // namespace mv

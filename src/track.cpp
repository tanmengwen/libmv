#include <json.h>
#include "mv.h"
#include "json_numeric.h"

using json::Json;
using json::NewMap;
using json::NewArray;

namespace mv {

const string& TrackedSequence::get_format() const {
	return format_;
}
void TrackedSequence::set_format(const string &fmt) {
	format_ = fmt;
}

Frame* TrackedSequence::add_frame()
{
	Frame *f = new Frame();
	assert(f);
	f->set_frame_number(size());
	push_back(f);

	return f;
}

/* mostly just convenience for testing; grows frames/tracks as needed */
Feature* TrackedSequence::add_point_feature(unsigned int frame, unsigned int track, double x, double y)
{
	if (frame >= size()) {
		unsigned int oldsize = size();
		resize(frame+1);
		for (unsigned int i=oldsize; i<size(); i++) {
			if (!at(i)) {
				Frame *f = at(i) = new Frame();
				assert(f);
				f->set_frame_number(i);
			}
		}
	}

	if (track >= tracks.size()) {
		unsigned int oldsize = tracks.size();
		tracks.resize(track+1);
		for (unsigned int i=oldsize; i<tracks.size(); i++) {
			if (!tracks[i]) {
				Track *t = tracks[i] = new PointTrack();
				assert(t);
				t->set_number(i);
			}
		}
	}

	Frame *f = at(frame);
	Track *t = tracks[track];
	Feature* point = new PointFeature(x,y);

	point->set_frame(f);
	point->set_frame_number(f->get_frame_number());
	point->set_number(f->size());
	point->set_track(t);

	f->push_feature(point);
	t->push_feature(point);

	return point;
}

string TrackedSequence::dump_cpp_ascii()
{
	string ret = "TrackedSequence ts;\n";
	for (size_t i=0; i<size(); i++) {
		Frame* frame = at(i);
		if (frame->size() != 0)
			for (size_t j=0; j<frame->size(); j++)
				ret += frame->at(j)->dump_cpp_ascii();

		if (frame->get_filename().size() != 0) {
			char buf[500];
			snprintf(buf,500,"ts[%d]->set_filename(\"%s\");\n",
					i, frame->get_filename().c_str());
			ret += string(buf);
		}
	}

	return ret;
}

Json* TrackedSequence::dump_json()
{
	number_items();

	Json *js = NewMap();
	js->set("type", "tracked sequence");
	js->set("num_frames", size());
	js->set("num_tracks", tracks.size());
	js->set("video_width", ncols);
	js->set("video_height", nrows);
	if (get_format().size())
		js->set("format", get_format());
	Json &jframes = js->set_new_array("frames");
	for (size_t i=0; i<size(); i++) {
		Json &jframe = jframes.append_new_map();
		Frame* frame = at(i);
		if (frame->size() != 0) {
			Json &jfeats = jframe.set_new_array("features");
			for (size_t j=0; j<frame->size(); j++) {
				jfeats.append(frame->at(j)->dump_json());
			}
		}
		if (frame->get_orig_ind() != -1)
			jframe.set("orig_ind", frame->get_orig_ind());

		if (frame->get_filename().size() != 0)
			jframe.set("filename", frame->get_filename());
	}

	return js;
}

TrackedSequence* json_to_tracked_sequence(Json* js)
{
	TrackedSequence *ts = new TrackedSequence;
	assert(js->get("type").as_string() == "tracked sequence");
	if (js->has_key("video_width"))
		ts->set_width(js->get("video_width").as_int());
	if (js->has_key("video_height"))
		ts->set_height(js->get("video_height").as_int());
	if (js->has_key("format") )
		ts->set_format(js->get("format").as_string());
	bool has_covariance = false;
	if (js->has_key("config") )
		if (js->get("config").has_key("do_unscented_transform") )
			has_covariance = js->get("config")["do_unscented_transform"].as_bool();

	Json &jframes = js->get("frames");
	for (size_t i=0; i<jframes.size(); i++) {
		Json &jfeatures = jframes[i]["features"];
		for (size_t j=0; j<jfeatures.size(); j++) {
			Json &jfeat = jfeatures[j];
			// FIXME assert that this is a point feature!
			PointFeature* pf = (PointFeature*) ts->add_point_feature(
					i,
					jfeat["track"].as_int(),
					jfeat["coords"][0].as_double(), 
					jfeat["coords"][1].as_double()); 
			// FIXME check if UKLT was used in tracking; if not don't load this
			if (has_covariance)
				json2mat(&jfeat["covariance"], &pf->sigma);

		}

		if (jframes[i].has_key("filename"))
			ts->at(i)->set_filename(jframes[i]["filename"].as_string());
		if (jframes[i].has_key("orig_ind"))
			ts->at(i)->set_orig_ind(jframes[i]["orig_ind"].as_int());
	}

	ts->number_items();

	return ts;
}

TrackedSequence *read_json_track(const char *fn) 
{
	Json* js = json::FromFile(fn);
	if (js == NULL)
		return NULL;
	TrackedSequence *ts = json_to_tracked_sequence(js);
	ts->number_items();
	delete js;
	return ts;
}

/*
TrackedSequence* trim_tracks_shorter_than(
		const TrackedSequence &ts,
		int min_track_length)
{
//	std::set<size_t> to_remove;
//	TrackedSequence *ret = new TrackedSequence();

	// find tracks to remove
//	for (size_t i=0; i<ts.tracks.size(); i++)
//		if (ts.tracks[i]->size() < min_track_length)
//			to_remove.insert(i);

	// 
//	for (int i=0; i<ts.size(); i++) {

//		for (int i=0; i<ts.size(); i++)
//			ret->add_point_feature(i, );
//	}
//	return ts2; // FIXME
}
*/

} // namespace mv

#include <json.h>
#include "mv.h"
#include "linalg.h"
#include "structure.h"
#include "camera.h"
using namespace json;

namespace mv {

Json* Reconstruction::dump_json() {
	number_items();

	Json &root = *NewMap();
	root.set("type", "base reconstruction");

	root.set_new_array("cameras");
	for (size_t i=0; i<cameras.size(); i++)
		root["cameras"].append(cameras[i]->dump_json());

	root.set_new_array("structure");
	for (size_t i=0; i<structure.size(); i++)
		root["structure"].append(structure[i]->dump_json());

	root.set_new_array("measurements");
	std::map<CToS, Feature*>::iterator m;
	for (m=measurements.begin(); m!=measurements.end(); ++m) {
		if (m->second == NULL)
			continue; // for testing, we want this so we don't always need a tracked sequence.
		CToS ctos = m->first;
		Camera* cam = ctos.first;
		Structure* st = ctos.second;
		int snum = st->get_number();
		int cnum = cam->get_number();
		Json& measurement = root["measurements"].append_new_map();
		measurement.set("cam_number", cnum);
		measurement.set("str_number", snum);
		measurement.set("feature_number", m->second->get_number());
	}
	return &root;
}

Reconstruction *json_to_reconstruction(TrackedSequence &ts, const Json &js) {
	if (!js.has_key("type"))
		return NULL;

	Reconstruction *ret = NULL;
	if (js["type"].as_string() == "projective reconstruction")
		ret = new ProjectiveReconstruction;
	else if (js["type"].as_string() == "euclidean reconstruction")
		ret = new EuclideanReconstruction;
	else
		return NULL;

	ret->set_tracked_sequence(&ts);

	for (size_t i=0; i<js["cameras"].size(); i++)
		ret->push_camera(json_to_camera(ts, js["cameras"][i]));

	// FIXME make this not depend on euc vs proj
	if (js["type"].as_string() == "projective reconstruction")
		for (size_t i=0; i<js["structure"].size(); i++)
			ret->push_structure(json_to_point_structure(ts, js["structure"][i]));
	else 
		for (size_t i=0; i<js["structure"].size(); i++)
			ret->push_structure(json_to_euclidean_point_structure(ts, js["structure"][i]));

	for (size_t i=0; i<js["measurements"].size(); i++) {
		Json &ms = js["measurements"][i];
		size_t cnum = ms["cam_number"].as_int();
		size_t snum = ms["str_number"].as_int();
		size_t fnum = ms["feature_number"].as_int();
		Frame* frame = ret->cameras[cnum]->get_frame();
		Feature *feat = frame->at(fnum);
		ret->push_measurement(
				ret->cameras[cnum],
				ret->structure[snum],
				feat);
	}

	ret->number_items();
	return ret;
}

Json* ProjectiveReconstruction::dump_json() {
	Json* root = Reconstruction::dump_json();
	root->set("type", "projective reconstruction");
	return root;
}

Json* EuclideanReconstruction::dump_json() {
	Json* root = Reconstruction::dump_json();
	root->set("type", "euclidean reconstruction");
	return root;
}

void json_to_reconstructions(TrackedSequence &ts, const Json &js,
		std::vector<Reconstruction*> *prs) {
	Json &reconstructions = js["reconstructions"];
	prs->resize(reconstructions.size());
	for (size_t i=0; i<reconstructions.size(); i++) {
		prs->at(i) = json_to_reconstruction(ts, reconstructions[i]);
	}
}
Json* reconstructions_to_json(const std::vector<Reconstruction*> &prs) {
	Json* ret = NewMap();
	Json& reconstructions = ret->set_new_array("reconstructions");
	for (size_t i=0; i<prs.size(); i++) {
		Json* json_reconstruction = prs[i]->dump_json();
		reconstructions.append(json_reconstruction);
	}
	return ret;
}

} // namespace mv

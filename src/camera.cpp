#include <json.h>
#include "mv.h"
#include "linalg.h"
#include "json_numeric.h"
using namespace json;

namespace mv {

mat opengl_to_normalized_k(const mat &ogl)
{
	mat k(eye(3));
	assert(ogl(0,0) == 1.0);
	assert(ogl(1,1) == 1.0);
	double f = 1.0/ogl(3,2);
	for (int i=0; i<2; i++)
		for (int j=0; j<3; j++)
			k(i,j) = ogl(i,j)*f;
	for (int j=0; j<3; j++)
		k(2,j) = ogl(3,j)*f;

	return k;
}

Json* ProjectiveCamera::dump_json() const {
	Json *js = NewMap();
	js->set("type", "projective camera");
	if (frame)
		js->set("frame", frame->get_frame_number());
	js->set("P", mat2json(*this));
	return js;
}

ProjectiveCamera* json_to_projective_camera(TrackedSequence &ts, Json* js) {
	ProjectiveCamera *cam = new ProjectiveCamera;
	Json &m = *js;
	assert(m["type"].as_string() == "projective camera");
	json2mat(&m["P"], cam);
	if (m.has_key("frame")) {
		size_t frame = m["frame"].as_int();
		cam->set_frame(ts[frame]);
	}
	return cam;
}

Json* EuclideanCamera::dump_json() const {
	Json *js = NewMap();
	js->set("type", "euclidean camera");
	if (frame)
		js->set("frame", frame->get_frame_number());
	js->set("K", mat2json(K));
	js->set("R", mat2json(R));
	js->set("t", vec2json(t));
	return js;
}

EuclideanCamera* json_to_euclidean_camera(TrackedSequence &ts, Json* js) {
	EuclideanCamera *cam = new EuclideanCamera;
	Json &m = *js;
	assert(m["type"].as_string() == "euclidean camera");
	json2mat(&m["K"], &cam->K);
	json2mat(&m["R"], &cam->R);
	json2vec(&m["t"], &cam->t);
	if (m.has_key("frame")) {
		size_t frame = m["frame"].as_int();
		cam->set_frame(ts[frame]);
	}
	return cam;
}

Camera *json_to_camera(TrackedSequence &ts, Json& js) {
	if (js["type"].as_string() == "euclidean camera") {
		return json_to_euclidean_camera(ts, &js);
	} else if (js["type"].as_string() == "projective camera") {
		return json_to_projective_camera(ts, &js);
	}
	return NULL;
}

bool Camera::base_equals(const Camera& other) const {
	if (frame != other.frame)
		return false;
	if (number != other.number)
		return false;
	return true;
}
bool Camera::equals(const ProjectiveCamera& other) const {
	(void) other;
	return false;
}
bool Camera::equals(const EuclideanCamera& other) const {
	(void) other;
	return false;
}

bool ProjectiveCamera::equals(const Camera &other) const {
	return other.equals(*this);
}

bool ProjectiveCamera::equals(const ProjectiveCamera &other) const {
	if (!base_equals(other))
		return false;
	for (size_t i=0; i<3; i++) 
		for (size_t j=0; j<4; j++) 
			if ((*this)(i,j) != other(i,j))
				return false;
	return true;
}

bool EuclideanCamera::equals(const Camera &other) const {
	return other.equals(*this);
}
bool EuclideanCamera::equals(const EuclideanCamera &other) const {
	if (!Camera::base_equals(other))
		return false;
	for (size_t i=0; i<3; i++) {
		for (size_t j=0; j<3; j++) {
			if (this->K(i,j) != other.K(i,j))
				return false;
			if (this->R(i,j) != other.R(i,j))
				return false;
		}
		if (this->t[i], other.t[i])
			return false;
	}
	return true;
}

} // namespace mv

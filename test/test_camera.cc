#include <testmv.h>
#include "camera.h"

TEST(opengl_perspective_to_normalized_k) {
	double oglmat[] = {
		1., 0., 0., 0.,
		0., 1., 0., 0.,
		0., 0., 4., 5.,
		0., 0., 1./3., 0.
	};
	mat ogl(4,4);
	fill(ogl, oglmat);
	double K[] = {
		3., 0., 0.,
		0., 3., 0.,
		0., 0., 1.
	};
	mat Km(3,3);
	fill(Km, K);
	mat ooo = opengl_to_normalized_k(ogl);
	Km -= ooo;
	AllVerySmall(Km);
}

TEST(projective_camera_dump_json) {
	ProjectiveCamera cam;
	Frame frame;
	frame.set_frame_number(3);
	cam.set_frame(&frame);
	Json &m = *cam.dump_json();
	Equals(m["type"].as_string(), "projective camera");
	Equals(m["frame"].as_int(), 3);
	Equals(m.has_key("P"), true);
}

TEST(euclidean_camera_dump_json) {
	EuclideanCamera cam;
	Frame frame;
	frame.set_frame_number(3);
	cam.set_frame(&frame);
	Json &m = *cam.dump_json();
	Equals(m["type"].as_string(), "euclidean camera");
	Equals(m["frame"].as_int(), 3);
	Equals(m.has_key("K"), true);
	Equals(m.has_key("R"), true);
	Equals(m.has_key("t"), true);
}

TEST(projective_camera_read_from_json) {
	TrackedSequence ts;
	ts.resize(2);
	Frame frame;
	frame.set_frame_number(3);
	ts[1] = &frame;
	Json *js = json::Parse(
			"{\"type\":\"projective camera\","
			" \"frame\":1,"
			" \"P\":[[9,2,3,4],[4,5,6,7],[7,8,9,10]]}"
			);
	ProjectiveCamera *cam = json_to_projective_camera(ts, js);
	Equals(cam->get_frame_number(), 3u);
	Equals((*cam)(0,0), 9); // only check top left element...
	Equals((*cam)(1,0), 4);
	Equals((*cam)(0,1), 2);
}

TEST(euclidean_camera_read_from_json) {
	TrackedSequence ts;
	ts.resize(2);
	Frame frame;
	frame.set_frame_number(3);
	ts[1] = &frame;
	Json *js = json::Parse(
			"{\"type\":\"euclidean camera\","
			" \"frame\":1,"
			" \"K\":[[1,2,3],[4,5,6],[7,8,9]],"
			" \"R\":[[9,9,9],[6,6,7],[1,1,1]],"
			" \"t\":[2,2,2]}");
	EuclideanCamera *cam = json_to_euclidean_camera(ts, js);
	Equals(cam->get_frame_number(), 3u);
	Equals(cam->K(0,0), 1); // only check top left element...
	Equals(cam->R(0,0), 9);
	Equals(cam->t[0], 2);
}

TEST(camera_equals_projective_simple) {
	ProjectiveCamera p1, p2;
	EuclideanCamera e1, e2;
	Equals(p1.equals(p2), true);
	p1(1,2) = 4.0;
	Equals(p1.equals(p2), false);
	Equals(e1.equals(p2), false);
	Equals(p1.equals(e2), false);
	Equals(e1.equals(e2), true);
	e1.K(1,2) = 4.0;
	Equals(e1.equals(e2), false);
	e2.K(1,2) = 4.0;
	Equals(e1.equals(e2), true);
}

TEST(camera_load_from_json_generic) {
	ProjectiveCamera p;
	EuclideanCamera e;
	TrackedSequence ts;
	Json &pj = *p.dump_json();
	Json &ej = *e.dump_json();
//	ProjectiveCamera *pjj = (ProjectiveCamera*)json_to_camera(ts, pj);
//	EuclideanCamera *ejj = (EuclideanCamera*)json_to_camera(ts, ej);
	Camera *pjj = json_to_camera(ts, pj);
	Camera *ejj = json_to_camera(ts, ej);
	Equals(pjj->equals(p), true);
	Equals(pjj->equals(e), false);
	std::cout << "E   " << e.dump_json()->to_string() << std::endl;
	std::cout << "EJJ " << ejj->dump_json()->to_string() << std::endl;
	Equals(ejj->equals(e), true);
	Equals(ejj->equals(p), false);
}

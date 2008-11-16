#include <testmv.h>
#include "json_numeric.h"

TEST(vector_serialize_simple) {
	vec x(3);
	x[0] = 5.0;
	x[1] = 6.0;
	x[2] = 7.0;
	Json* js = vec2json(x);
	Equals(js->to_string(), "[5,6,7]");
	delete js;
}

TEST(matrix_serialize_simple) {
	mat m(2,3);
	m(0,0) = 5.0;
	m(0,1) = 6.0;
	m(0,2) = 7.0;
	m(1,0) = 7.0;
	m(1,1) = 8.0;
	m(1,2) = 9.0;
	Json* js = mat2json(m);
	Equals(js->to_string(), "[[5,6,7],[7,8,9]]");
	delete js;
}

TEST(vector_unserialize) {
	vec x(3);
	Json* js = json::Parse("[3,1,4]");
	json2vec(js, &x);
	Equals(x[0], 3);
	Equals(x[1], 1);
	Equals(x[2], 4);
	delete js;
}

TEST(matrix_unserialize) {
	mat m(2,3);
	Json* js = json::Parse("[[5,6,7],[7,8,12]]");
	json2mat(js, &m);
	Equals(m(0,0), 5.0);
	Equals(m(0,1), 6.0);
	Equals(m(0,2), 7.0);
	Equals(m(1,0), 7.0);
	Equals(m(1,1), 8.0);
	Equals(m(1,2), 12.0);
	delete js;
}

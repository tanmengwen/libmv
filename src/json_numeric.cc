#include "mv.h"
#include "linalg.h"
#include <json.h>
using namespace json;

namespace mv {

Json* vec2json(const vec& v) {
	Json *arr = NewArray();
	for (size_t i=0; i<v.size(); i++)
		arr->append(v[i]);
	return arr;
}

Json* mat2json(const mat& m) {
	Json *arr = NewArray();
	for (size_t i=0; i<m.size1(); i++) {
		Json &row = arr->append_new_array();
		for (size_t j=0; j<m.size2(); j++)
			row.append(m(i,j));
	}
	return arr;
}

void json2vec(Json *js, vec *xx) {
	size_t length = js->size();
	vec &x = *xx;
	x.resize(length);
	for (size_t i=0; i<length; i++)
		x[i] = js->get(i).as_double();
}

void json2mat(Json *js, mat *mm) {
	Json &arr = *js;
	size_t rows = arr.size();
	size_t cols = arr[0].size();
	mat &m = *mm;
	m.resize(rows,cols);
	for (size_t i=0; i<rows; i++) {
		for (size_t j=0; j<cols; j++) {
			m(i,j) = arr[i][j].as_double();
		}
	}
}

bool json_to_file(const Json &js, const char *fn) {
	FILE* fid = fopen(fn, "w");
	if (!fid)
		return false;
	string out = js.to_string();
	fprintf(fid, "%s", out.c_str());
	fclose(fid);
	return true;
}

Json* file_to_json(const char *fn) {
	FILE* fid = fopen(fn, "r");
	if (!fid)
		return false;
	Json *ret = json::Parse(fid);
	fclose(fid);
	return ret;
}

} // namespace mv

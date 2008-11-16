#ifndef _JSON_NUMERIC_H
#define _JSON_NUMERIC_H

#include "mv.h"

namespace mv {

Json* vec2json(const vec& v);
Json* mat2json(const mat& m);

void json2vec(Json *js, vec *v);
void json2mat(Json *js, mat *m);
bool json_to_file(const Json &js, const char *fn);
Json* file_to_json(const char *fn);

}

#endif // _JSON_NUMERIC_H

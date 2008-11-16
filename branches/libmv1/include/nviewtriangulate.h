#ifndef _NVIEWTRIANGULATE_H
#define _NVIEWTRIANGULATE_H

#include "mv.h" 

namespace mv {

void nviewtriangulate(
		mat &points,
		std::vector<mat*> cameras,
		vec *X_out);

}

#endif // _NVIEWTRIANGULATE_H

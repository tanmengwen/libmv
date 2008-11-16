#ifndef _RESECTION_H
#define _RESECTION_H

#include "mv.h" 
#include "ransac.h" 

namespace mv {

void camera_resection(const mat &x, const mat &X, mat *P);
bool robust_camera_resection(const mat& x, const mat& X, mat *P, double *score);

}

#endif // _RESECTION_H

#include "mv.h"

namespace mv {

// Points is 
void nviewtriangulate(
		mat &points,
		std::vector<mat*> cameras,
		vec *X_out)
{
	size_t nviews = points.size1();

	mat transposed_points(3, nviews);
	for (size_t i=0; i<nviews; i++) {
		transposed_points(0, i) = points(i, 0);
		transposed_points(1, i) = points(i, 1);
		transposed_points(2, i) = 1.0;
	}
	mat H = normalizing_homography(transposed_points);
	mat normalized_points = dot(H, transposed_points);

	mat design(3*nviews, 4+nviews);
	fill(design, 0.0);
	for (size_t i=0; i<nviews; i++) {

		mat normalized_camera = dot(H, (*(cameras[i])));
		for (int r=0; r<3; r++)
			for (int c=0; c<4; c++)
				design(3*i+r, c) = -normalized_camera(r,c);

		design(3*i+0, 4+i) = normalized_points(0,i);
		design(3*i+1, 4+i) = normalized_points(1,i);
		design(3*i+2, 4+i) = normalized_points(2,i);
	}
	vec X_and_alphas;
	nullspace(design, X_and_alphas);
	X_out->resize(4);
	for (size_t i=0; i<4; i++)
		(*X_out)[i] = X_and_alphas[i];
}

} // namespace mv

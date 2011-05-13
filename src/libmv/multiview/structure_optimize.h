// Copyright (c) 2011 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef LIBMV_MULTIVIEW_STRUCTURE_OPTIMIZE_H_
#define LIBMV_MULTIVIEW_STRUCTURE_OPTIMIZE_H_

#include "libmv/multiview/camera_error.h"
#include "libmv/multiview/structure_optimize_functor.h"

namespace libmv {
namespace point3D {

/**
 * Computes the optimal points coordinates from their images and a
 * projection matrix. The method use an euclidean parameterization
 * of the 3D points.
 *
 * \param[in] x_image  Image points coordinates (in pixels)
 * \param[in] Ps       A vector of camera projection matrices associated 
                       to each 2D point
 * \param[in/out] X_world   Initial/Optimal 3D points in the world coordinate system
 * \param[in] max_iter The maximun number of iterations of the nonlinear method
 */
void OptimizeEuclideanPoints(const Mat2X &x_image, 
                             const vector<Mat34> &Ps, 
                             Mat3X *X_world,
                             int max_iter = 400) {
  typedef Analytic2RowsFunctor<EuclideanParameterization<>, 
                               GeometricError> StructureFunctor;
  int info;
  Vec x;
  EuclideanParameterization<double>::From(*X_world, &x);
  StructureFunctor structure_functor(Ps, *X_world, x_image);
  Eigen::LevenbergMarquardt<StructureFunctor> lm(structure_functor);
  lm.set_max_iter(max_iter);
  info = lm.minimize(x);  
  
  EuclideanParameterization<double>::To(x, *X_world);
  return info > 0;
}

/**
 * Computes the optimal points coordinates from their images and a
 * projection matrix. The method use a homogeneous parameterization
 * of the 3D points.
 *
 * \param[in] x_image  Image points coordinates (in pixels)
 * \param[in] Ps       A vector of camera projection matrices associated 
                       to each 2D point
 * \param[in/out] X_world   Initial/Optimal 3D points in the world coordinate system
 * \param[in] max_iter The maximun number of iterations of the nonlinear method
 */
void OptimizeHomogeneousPoints(const Mat2X &x_image, 
                               const vector<Mat34> &Ps, 
                               Mat4X *X_world,
                               int max_iter = 400) {
  typedef Analytic2RowsFunctor<HomogeneousParameterization<>, 
                               GeometricError> StructureFunctor;
  int info;
  Vec x;
  HomogeneousParameterization<double>::From(*X_world, &x);
  StructureFunctor structure_functor(Ps, *X_world, x_image);
  Eigen::LevenbergMarquardt<StructureFunctor> lm(structure_functor);
  lm.set_max_iter(max_iter);
  info = lm.minimize(x);  
  
  HomogeneousParameterization<double>::To(x, *X_world);
  return info > 0;
}

}  // namespace point3D
}  // namespace libmv

#endif  // LIBMV_MULTIVIEW_STRUCTURE_OPTIMIZE_H_

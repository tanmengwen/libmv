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

#ifndef LIBMV_MULTIVIEW_CAMERA_OPTIMIZE_H_
#define LIBMV_MULTIVIEW_CAMERA_OPTIMIZE_H_

#include "libmv/multiview/camera_optimize_functor.h"
#include "libmv/multiview/euclidean_parameterization.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace camera {
namespace pinhole {
namespace calibrated {

/**
 * Computes the optimal extrinsic parameters, R and t for a calibrated camera
 * from 4 or more 3D points and their images.
 *
 * \param[in] x_camera  Image points in normalized camera coordinates
 *                      e.g. x_camera=inv(K)*x_image
 * \param[in] X_world   3D points in the world coordinate system
 * \param[in/out] R     Initial/Optimal solution of the camera rotation matrix
 * \param[in/out] t     Initial/Optimal solution of the camera translation vector
 * \param[in] max_iter The maximun number of iterations of the nonlinear method
 */
void OptimizeCamera(const Mat2X &x_camera, 
                    const Mat3X &X_world,
                    Mat3 *R, Vec3 *t,
                    int max_iter = 400) {
  typedef Analytic2RowsFunctor<Euclidean2DEulerParameterization<>, 
                               GeometricError> CameraFunctor;
  int info;
  // NOTE, this is a relative angles parametrization so we start at 0.
  Vec x(6); x << 0, 0, 0, *t;
  Mat4 C; C.setIdentity();
  C.block<3,3>(0,0)  = *R;
  C.col(3).head<3>() = *t;
  Mat3 K; K.setIdentity();
  
  CameraFunctor camera_functor(K, C, X_world, x_camera);
  Eigen::LevenbergMarquardt<CameraFunctor> lm(camera_functor);
  lm.set_max_iter(max_iter);
  info = lm.minimize(x);  
  
  Euclidean2DEulerParameterization<double>::To(x, &C);
  *R *= C.block<3,3>(0,0);
  *t = C.col(3).head<3>();
  return info > 0;
}

} // namespace calibrated
namespace uncalibrated {

/**
 * Computes the optimal projection matrix of an uncalibrated affine camera
 * from 4 or more 3D points and their images.
 *
 * \param[in] x_image  Image points coordinates (in pixels)
 * \param[in] X_world   3D points in the world coordinate system
 * \param[in/out] P     Initial/Optimal solution of the camera projection matrix
 * \param[in] max_iter The maximun number of iterations of the nonlinear method
 */
void OptimizeAffineCamera(const Mat2X &x_image, 
                          const Mat4X &X_world,
                          Mat34 *P, int max_iter = 400) {
  typedef Analytic2RowsFunctor<AffineCameraParameterization<>, 
                               GeometricError> CameraFunctor;
  int info;
  Vec8 x8;
  ProjectiveCameraParameterization<double>::From(*P, &x8);
  CameraFunctor camera_functor(*P, X_world, x_camera);
  Eigen::LevenbergMarquardt<CameraFunctor> lm(camera_functor);
  lm.set_max_iter(max_iter);
  Vec x = x8; // TODO(julien) avoid this copy
  info = lm.minimize(x);  
  
  AffineCameraParameterization<double>::To(x, *P);
  return info > 0;
}

/**
 * Computes the optimal projection matrix of an uncalibrated projective camera
 * from 6 or more 3D points and their images.
 *
 * \param[in] x_image  Image points coordinates (in pixels)
 * \param[in] X_world   3D points in the world coordinate system
 * \param[in/out] P     Initial/Optimal solution of the camera projection matrix
 * \param[in] max_iter The maximun number of iterations of the nonlinear method
 */
void OptimizeProjectiveCamera(const Mat2X &x_image, 
                              const Mat4X &X_world,
                              Mat34 *P, int max_iter = 400) {
  typedef Analytic2RowsFunctor<ProjectiveCameraParameterization<>, 
                               GeometricError> CameraKer;
  int info;
  Vec11 x11;
  ProjectiveCameraParameterization<double>::From(*P, &x11);
  CameraKer camera_functor(*P, X_world, x_camera);
  Eigen::LevenbergMarquardt<CameraKer> lm(camera_functor);
  lm.set_max_iter(max_iter);
  Vec x = x11; // TODO(julien) avoid this copy
  info = lm.minimize(x);  
  
  ProjectiveCameraParameterization<double>::To(x, *P);
  return info > 0;
}
} // namespace pinhole
} // namespace uncalibrated
} // namespace camera
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_CAMERA_OPTIMIZE_H_

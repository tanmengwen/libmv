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

#ifndef LIBMV_MULTIVIEW_PROJECTION_ERRORS_H_
#define LIBMV_MULTIVIEW_PROJECTION_ERRORS_H_

#include "libmv/multiview/projection.h"

namespace libmv {
namespace projection {

 /**
   * Structure for estimating the geometric error between a 3D point X and a 
   * 2D point x such that 
   *   Error = ||x - Psi(P * X)||^2
   * where Psi is the function that transforms homogeneous to euclidean coords.
   * \note It should be distributed as Chi-squared with k = 2.
   */
struct GeometricError {
  /**
   * Computes the geometric residuals between a set of 3D points X and a set of
   * 2D points x such that 
   *   Errori = ||xi - Psi(P * Xi)||^2
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated homography should approximatelly hold the condition y = P x.
   * \param[in]  X  A 3xN or 4xN matrix of column vectors.
   * \param[in]  x  A 2xN or 3xN matrix of column vectors.
   * \param[out] dx  A 3xN matrix of column vectors of residuals errors
   */
  static void Residuals(const Mat34 &P, const Mat &X, 
                        const Mat &x, Mat3X *dx) {
    dx->resize(3, x1.cols());
    if (X2.rows() == 3)
      *dx = X2 - Project(P, X1);
    else
      *dx = HomogeneousToEuclidean(static_cast<Mat4X>(X2)) - Project(P, X1);
  }
  /**
   * Computes the residuals between X2 and the transformed X1 such that
   *   residuals = X2 - Psi(P * X1) 
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated homography should approximatelly hold the condition y = P x.
   * \param[in]  X  A vector of size 3 (euclidean) or 4 (homogeneous) 
   * \param[in]  x  A vector of size 3 (euclidean) or 4 (homogeneous) 
   * \param[out] dx  A vector of size 2 of the residual error
   */
  static void Residuals(const Mat &P34, const Vec &X, 
                        const Vec &x, Vec3 *dx) {
    Vec3 X2h_est;
    Project(P, X1, &X2h_est);
    if (X2.rows() == 2)
      *dx = X2 - X2h_est;
    else
      *dx = HomogeneousToEuclidean(static_cast<Vec4>(X2)) - X2h_est;
  }
  /**
   * Computes the squared norm of the residuals between X2 and the 
   * transformed X1 such that  Error = || X2 - Psi(P * X1) ||^2 
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated homography should approximatelly hold the condition y = P x.
   * \param[in]  X A 3xN or 4xN matrix of column vectors.
   * \param[in]  x A 3xN or 4xN matrix of column vectors.
   * \return  The squared norm of the asymmetric residuals errors
   */
  static double Error(const Mat34 &P, const Mat &X, const Mat &x) {
    Mat3X dx;
    Residuals(P, X, x, &dx);
    return dx.squaredNorm();
  }
  /**
   * Computes the squared norm of the residuals between X2 and the 
   * transformed X1 such that  rms = || X2 - Psi(P * X1) ||^2 
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated homography should approximatelly hold the condition y = P x.
   * \param[in]  X  A vector of size 3 (euclidean) or 4 (homogeneous) 
   * \param[in]  x  A vector of size 3 (euclidean) or 4 (homogeneous) 
   * \return  The squared norm of the asymmetric residual error
   */
  static double Error(const Mat34 &P, const Vec &X, const Vec &x) {
    Vec3 dx;
    Residuals(P, X, x, &dx);
    return dx.squaredNorm();
  }
};

 /**
   * Structure for estimating the algebraic error (cross product) 
   * between a vector X2 and the transformed X1 such that 
   *   Error = ||[X2] * P * X1||^^2
   * where [X2] is the skew matrix of X2.
   */
struct AlgebraicError {
  /**
   * Computes the algebraic residuals (cross product) between X2 and 
   * the transformed X1 such that 
   *   [X2] * P * X1  where [X2] is the skew matrix of X2.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated homography should approximatelly hold the condition y = P x.
   * \param[in]  X1  A 3xN or 4xN matrix of column vectors.
   * \param[in]  X2  A 3xN or 4xN matrix of column vectors.
   * \param[out] dX  A 4xN matrix of column vectors of residuals errors
   */
  static void Residuals(const Mat34 &P, const Mat &X, 
                        const Mat &x, Mat4X *dx) {
    dx->resize(4, X1.cols());
    Vec4 col(4);
    for (int i = 0; i < X1.cols(); ++i) {
      Residuals(P, X.col(i), x.col(i), &col);
      dx->col(i) = col;
    }
  }
  /**
   * Computes the algebraic residuals (cross product) between X2 and 
   * the transformed X1 such that 
   *   [X2] * P * X1  where [X2] is the skew matrix of X2.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated homography should approximatelly hold the condition y = P x.
   * \param[in]  X1  A vector of size 3 (euclidean) or 4 (homogeneous) 
   * \param[in]  X2  A vector of size 3 (euclidean) or 4 (homogeneous) 
   * \param[out] dx  A vector of size 4 of the residual error
   */
  static void Residuals(const Mat34 &P, const Vec &X, 
                        const Vec &X, Vec4 *dx) {
    Vec4 X2h_est;
    if (X1.rows() == 3)
      X2h_est = P * EuclideanToHomogeneous(static_cast<Vec3>(X1));
    else
      X2h_est = P * X1;
    if (X2.rows() == 3)
      *dx = SkewMat(EuclideanToHomogeneous(static_cast<Vec3>(X2))) * X2h_est;
    else
      *dx = SkewMat(X2) * X2h_est;
    // TODO(julien) This is inefficient since it creates an
    // identical 3x3 skew matrix for each evaluation.
  }
  /**
   * Computes the squared norm of the algebraic residuals between X2 and the 
   * transformed X1 such that [X2] * P * X1  where [X2] is the skew matrix of X2.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated homography should approximatelly hold the condition y = P x.
   * \param[in]  X A 3xN or 4xN matrix of column vectors.
   * \param[in]  x A 3xN or 4xN matrix of column vectors.
   * \return  The squared norm of the asymmetric residuals errors
   */
  static double Error(const Mat34 &P, const Mat &X, const Mat &x) {
    Mat4X dx;
    Residuals(P, X, x, &dx);
    return dx.squaredNorm();
  }
  /**
   * Computes the squared norm of the algebraic residuals between X2 and the 
   * transformed X1 such that [X2] * P * X1  where [X2] is the skew matrix of X2.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated homography should approximatelly hold the condition y = P x.
   * \param[in]  X  A vector of size 3 (euclidean) or 4 (homogeneous) 
   * \param[in]  x  A vector of size 3 (euclidean) or 4 (homogeneous) 
   * \return  The squared norm of the asymmetric residual error
   */
  static double Error(const Mat34 &P, const Vec &X, const Vec &x) {
    Vec4 dx;
    Residuals(P, X, x, &dx);
    return dx.squaredNorm();
  }
};
// TODO(julien) Add here M-estimators (Tukey, etc.)
} // namespace projection {
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_PROJECTION_ERRORS_P_

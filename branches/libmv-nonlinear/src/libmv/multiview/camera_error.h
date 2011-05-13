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

#ifndef LIBMV_MULTIVIEW_CAMERA_ERRORS_H_
#define LIBMV_MULTIVIEW_CAMERA_ERRORS_H_

#include "libmv/multiview/projection.h"

namespace libmv {
namespace camera {
namespace pinhole {

 /**
   * Structure for estimating the geometric error between a projected 3D point Q 
   * and a 2D point q such that 
   *   Error = ||q - Psi(P * Q)||^2
   * where Psi is the function that transforms homogeneous to euclidean coords.
   * \note It should be distributed as Chi-squared with k = 2.
   */
struct GeometricError {
  /**
   * Computes the geometric residuals between a set of 3D points Q and a set of
   * 2D points q such that 
   *   Residual_i = ||x_i - Psi(P * X_i)||^2
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  Ps A vector of 3x4 projection matrices.
   * The estimated projection should approximatelly hold the condition q = Psi(P Q).
   * \param[in]  Q A set of 3D points (3xN or 4xN matrix of column vectors).
   * \param[in]  q A set of 2D points (2xN or 3xN matrix of column vectors).
   * \param[out] dx  A 2xN matrix of column vectors of residuals errors
   */
  static void Residuals(const vector<Mat34> &Ps, const Mat &Q, 
                        const Mat &q, Mat2X *dx) {
    dx->resize(2, q.cols());
    for (int i = 0; i < q.cols(); ++i) {
      Residuals(Ps[i], Q.col(i), q.col(i), &dx->col(i));
    }
  }
  /**
   * Computes the geometric residuals between a set of 3D points Q and a set of
   * 2D points q such that 
   *   Residual_i = ||x_i - Psi(P * X_i)||^2
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated projection should approximatelly hold the condition q = Psi(P Q).
   * \param[in]  Q A set of 3D points (3xN or 4xN matrix of column vectors).
   * \param[in]  q A set of 2D points (2xN or 3xN matrix of column vectors).
   * \param[out] dx  A 2xN matrix of column vectors of residuals errors
   */
  static void Residuals(const Mat34 &P, const Mat &Q, 
                        const Mat &q, Mat2X *dx) {
    dx->resize(2, q.cols());
    if (q.rows() == 2)
      *dx = q - Project(P, Q);
    else
      *dx = HomogeneousToEuclidean(static_cast<Mat3X>(q)) - Project(P, Q);
  }
  /**
   * Computes the residuals between X2 and the transformed X1 such that
   *   Residuals = q - Psi(P * Q) 
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated projection should approximatelly hold the condition q = Psi(P Q).
   * \param[in]  Q A 3D point (vector of size 3 or 4 (euclidean/homogeneous)) 
   * \param[in]  q A 2D point (vector of size 2 or 3 (euclidean/homogeneous)) 
   * \param[out] dx  A vector of size 2 of the residual error
   */
  static void Residuals(const Mat &P34, const Vec &Q, 
                        const Vec &q, Vec2 *dx) {
    Vec2 X_proj_h;
    Project(P, Q, &X_proj_h);
    if (q.rows() == 2)
      *dx = q - X_proj_h;
    else
      *dx = HomogeneousToEuclidean(static_cast<Vec3>(q)) - X_proj_h;
  }
  /**
   * Computes the squared norm of the asymmetric residuals between a set of projected 
   * 3D points Q and a set of 2D points q such that 
   * Error = || q - Psi(P * Q) ||^2 
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated projection should approximatelly hold the condition q = Psi(P Q).
   * \param[in]  Q A set of 3D points (3xN or 4xN matrix of column vectors).
   * \param[in]  q A set of 2D points (2xN or 3xN matrix of column vectors).
   * \return  The squared norm of the asymmetric residuals errors
   */
  static double Error(const Mat34 &P, const Mat &Q, const Mat &q) {
    Mat2X dx;
    Residuals(P, Q, q, &dx);
    return dx.squaredNorm();
  }
  /**
   * Computes the squared norm of the asymmetric residuals between a projected 
   * 3D point Q and a 2D point q such that  rms = || X2 - Psi(P * X1) ||^2 
   * where Psi is the function that transforms homogeneous to euclidean coords.
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated projection should approximatelly hold the condition q = Psi(P Q).
   * \param[in]  Q A 3D point (vector of size 3 or 4 (euclidean/homogeneous)) 
   * \param[in]  q A 2D point (vector of size 2 or 3 (euclidean/homogeneous)) 
   * \return  The squared norm of the asymmetric residual error
   */
  static double Error(const Mat34 &P, const Vec &Q, const Vec &q) {
    Vec2 dx;
    Residuals(P, Q, q, &dx);
    return dx.squaredNorm();
  }
};

 /**
   * Structure for estimating the algebraic error between a projected 3D point Q 
   * and a 2D point q such that 
   *   Error = ||[q] * P * Q||^2
   * where [q] is the skew matrix of q.
   */
struct AlgebraicError {
  /**
   * Computes the algebraic residuals (cross product) between a set of projected 
   * 3D points Q and a set of 2D points q such that 
   *   Residuals_i = [x_i] * H *  X_i
   * where [q] is the skew matrix of q (matrix cross product)
   *
   * \param[in]  Ps A vector of 3x4 projection matrices.
   * The estimated projection should approximatelly hold the condition q = Psi(P Q).
   * \param[in]  Q A set of 3D points (3xN or 4xN matrix of column vectors).
   * \param[in]  q A set of 2D points (2xN or 3xN matrix of column vectors).
   * \param[out] dX  A 3xN matrix of column vectors of algebraic residuals errors
   */
  static void Residuals(const vector<Mat34> &Ps, const Mat &Q, 
                        const Mat &q, Mat2X *dx) {
    dx->resize(2, q.cols());
    for (int i = 0; i < q.cols(); ++i) {
      Residuals(Ps[i], Q.col(i), q.col(i), &dx->col(i));
    }
  }
  /**
   * Computes the algebraic residuals (cross product) between a set of projected 
   * 3D points Q and a set of 2D points q such that 
   *   Residuals_i = [x_i] * H *  X_i
   * where [q] is the skew matrix of q (matrix cross product)
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated projection should approximatelly hold the condition q = Psi(P Q).
   * \param[in]  Q A set of 3D points (3xN or 4xN matrix of column vectors).
   * \param[in]  q A set of 2D points (2xN or 3xN matrix of column vectors).
   * \param[out] dX  A 3xN matrix of column vectors of algebraic residuals errors
   */
  static void Residuals(const Mat34 &P, const Mat &Q, 
                        const Mat &q, Mat3X *dx) {
    dx->resize(3, X1.cols());
    Vec3 col;
    for (int i = 0; i < X1.cols(); ++i) {
      Residuals(P, Q.col(i), q.col(i), &col);
      dx->col(i) = col;
    }
  }
  /**
   * Computes the algebraic residuals (cross product) between a set of projected 
   * 3D points Q and a set of 2D points q such that 
   *   Residuals_i = |1 0 0| * [x_i] * H *  X_i
   *                 |0 1 0|
   * where [q] is the skew matrix of q (matrix cross product)
   *
   * \param[in]  P The 3x4 projection matrix.
   * The estimated projection should approximatelly hold the condition q = Psi(P Q).
   * \param[in]  Q A set of 3D points (3xN or 4xN matrix of column vectors).
   * \param[in]  q A set of 2D points (2xN or 3xN matrix of column vectors).
   * \param[out] dX  A 3xN matrix of column vectors of algebraic residuals errors
   */
  static void Residuals(const Mat34 &P, const Mat &Q, 
                        const Mat &q, Mat2X *dx) {
    dx->resize(2, X1.cols());
    Vec3 col;
    for (int i = 0; i < X1.cols(); ++i) {
      Residuals(P, Q.col(i), q.col(i), &col);
      dx->col(i) << col(0), col(1);
    }
  }
  /**
   * Computes the algebraic residuals (cross product) between a projected 
   * 3D point Q and a 2D point q such that 
   *   Residuals = [q] * H *  Q
   * where [q] is the skew matrix of q (matrix cross product)
   *
   * \param[in]  P The 3x4 projection matrix.
   * \param[in]  Q A 3D point (vector of size 3 or 4 (euclidean/homogeneous)) 
   * \param[in]  q A 2D point (vector of size 2 or 3 (euclidean/homogeneous)) 
   * \param[out] dx  A vector of size 3 of the algebraic residual error
   */
  static void Residuals(const Mat34 &P, const Vec &Q, 
                        const Vec &q, Vec3 *dx) {
    Vec3 X_proj_h;
    if (Q.rows() == 3)
      X_proj_h = P * EuclideanToHomogeneous(static_cast<Vec3>(Q));
    else
      X_proj_h = P * Q;
    if (q.rows() == 2)
      *dx = SkewMat(EuclideanToHomogeneous(static_cast<Vec2>(q))) * X_proj_h;
    else
      *dx = SkewMat(q) * X_proj_h;
    // TODO(julien) This is inefficient since it creates an
    // identical 3x3 skew matrix for each evaluation.
  }
  /**
   * Computes the squared norm of the algebraic residuals between a set of projected 
   * 3D points Q and a set of 2D points q such that  Error =  ||[q] * P * Q ||^2
   * where [q] is the skew matrix of q.
   *
   * \param[in]  P The 3x4 projection matrix.
   * \param[in]  Q A set of 3D points (3xN or 4xN matrix of column vectors).
   * \param[in]  q A set of 2D points (2xN or 3xN matrix of column vectors).
   * \return  The squared norm of the algebraic residuals errors
   */
  static double Error(const Mat34 &P, const Mat &Q, const Mat &q) {
    Mat3X dx;
    Residuals(P, Q, q, &dx);
    return dx.squaredNorm();
  }
  /**
   * Computes the squared norm of the algebraic residuals between a projected 
   * 3D point Q and a 2D point q such that Error = ||[q] * P * Q ||^2
   * where [q] is the skew matrix of q.
   *
   * \param[in]  P The 3x4 projection matrix.
   * \param[in]  Q A 3D point (vector of size 3 or 4 (euclidean/homogeneous)) 
   * \param[in]  q A 2D point (vector of size 2 or 3 (euclidean/homogeneous)) 
   * \return  The squared norm of the algebraic residual error
   */
  static double Error(const Mat34 &P, const Vec &Q, const Vec &q) {
    Vec3 dx;
    Residuals(P, Q, q, &dx);
    return dx.squaredNorm();
  }
};
// TODO(julien) Add here M-estimators (Tukey, etc.)
} // namespace pinhole
} // namespace camera
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_CAMERA_ERRORS_H_

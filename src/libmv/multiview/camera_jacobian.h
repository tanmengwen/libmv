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

#ifndef LIBMV_MULTIVIEW_CAMERA_JACOBIAN_H_
#define LIBMV_MULTIVIEW_CAMERA_JACOBIAN_H_

#include "libmv/multiview/camera_error.h"
#include "libmv/multiview/camera_parameterization.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace camera {
namespace pinhole {

/** A generic structure used only when the association of a parameterization with 
 * an error model is unknown.
 */
template <typename PARAMETERIZATION,
          typename ERROR>
struct Jacobian {
  /** Estimates the 2D jacobian evaluation of a vector of parameters q, for a
   * given cost function and set of static parameters Q, q.
   */
  static void Jac(const Mat34 &P, const Vec3 &Q, const Vec2 &q, Mat2X *jac) {
    YOU_ARE_USING_AN_UNDEFINED_JACOBIAN;
  }
  /** Estimates the 3D jacobian evaluation of a vector of parameters q, for a
   * given cost function and set of static parameters Q, q.
   */
  static void Jac(const Mat34 &P, const Vec3 &Q, const Vec2 &q, Mat3X *jac) {
    YOU_ARE_USING_AN_UNDEFINED_JACOBIAN;
  }
};

/** A structure used for defining the jacobian of a normalized 2D homography  
 * parameterization with the 2 rows geometric error model.
 */
template <>
struct Jacobian<ProjectiveCameraParameterization<>,
                GeometricError>{
  /** Return the 2D jacobian evaluation of a vector of parameters q, for the
   * algebraic error model and with a set of static parameters Q, q.
   */
  static void Jac(const Mat34 &P, const Vec3 &Q, const Vec2 &q, Mat2X *jac) {
    //TODO(julien);
  }
};

/** A structure used for defining the jacobian of a normalized 2D homography  
 * parameterization with the 2 rows algebraic error model.
 */
template <>
struct Jacobian<ProjectiveCameraParameterization<>,
                AlgebraicError>{
  /** Return the 2D jacobian evaluation of a vector of parameters q, for the
   * algebraic error model and with a set of static parameters Q, q.
   */
  static void Jac(const Mat34 &P, const Vec3 &Q, const Vec2 &q, Mat2X *jac) {
    //TODO(julien);
  }
};

} // namespace pinhole
} // namespace camera
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_CAMERA_JACOBIAN_H_

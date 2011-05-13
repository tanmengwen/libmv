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

#ifndef LIBMV_MULTIVIEW_AFFINE_JACOBIAN_H_
#define LIBMV_MULTIVIEW_AFFINE_JACOBIAN_H_

#include "libmv/multiview/homography_error.h"
#include "libmv/multiview/affine_parameterization.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace affine {
using namespace homography;
namespace affine2D {
using namespace homography2D;

/** A structure used for defining the jacobian of a generic 2D affine  
 * parameterization with the 2 rows asymmetric geometric error model.
 */
template <>
struct Jacobian<Affine2DGenericParameterization<>,
                AsymmetricError>{
  /** Return the 2D jacobian evaluation of a vector of parameters x, for the
   * 2 rows asymmetric geometric error model and with a set of static 
   * parameters x1, x2.
   */
  static void Jac(const Vec &x, const Vec3 &x1, const Vec3 &x2, Mat2X *jac) {
    // Errors on x
    *jac << 1,            //a
            1,            //b
            0,            //c
            0,            //d
            1,            //x
            0,            //y
    // Errors on y    
            0,            //a
            0,            //b
            1,            //c
            1,            //d
            0,            //x
            1;            //y
  }
};
} // namespace affine2D
} // namespace affine
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_AFFINE_JACOBIAN_H_

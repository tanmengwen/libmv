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

#ifndef LIBMV_MULTIVIEW_HOMOGRAPHY_JACOBIAN_H_
#define LIBMV_MULTIVIEW_HOMOGRAPHY_JACOBIAN_H_

#include "libmv/multiview/homography_error.h"
#include "libmv/multiview/homography_parameterization.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace homography {
namespace homography2D {

/** A generic structure used only when the association of a parameterization with 
 * an error model is unknown.
 */
template <typename PARAMETERIZATION,
          typename ERROR>
struct Jacobian {
  /** Return the 2D jacobian evaluation of a vector of parameters x, for a
   * given cost function and set of static parameters x1, x2.
   */
  static void Jac(const Vec &x, const Vec3 &x1, const Vec3 &x2, Mat2X *jac) {
    YOU_ARE_USING_AN_UNDEFINED_JACOBIAN;
  }
  
  /** Return the 3D jacobian evaluation of a vector of parameters x, for a
   * given cost function and set of static parameters x1, x2.
   */
  static void Jac(const Vec &x, const Vec3 &x1, const Vec3 &x2, Mat3X *jac) {
    YOU_ARE_USING_AN_UNDEFINED_JACOBIAN;
  }
};

/** A structure used for defining the jacobian of a normalized 2D homography  
 * parameterization with the 2 rows algebraic error model.
 */
template <>
struct Jacobian<Homography2DNormalizedParameterization<>,
                AlgebraicError>{
  /** Return the 2D jacobian evaluation of a vector of parameters x, for the
   * 2 rows algebraic error model and with a set of static parameters x1, x2.
   */
  static void Jac(const Vec &x, const Vec3 &x1, const Vec3 &x2, Mat2X *jac) {
    // Errors on x
    *jac << 0,                  //a
            0,                  //b
            0,                  //c
           -x1(0),              //d
           -x1(1),              //e
           -1,                  //f
            x2(1)*x1(0),        //g 
            x2(1)*x1(1),        //h 
    // Errors on y    
            x1(0),              //a
            x1(1),              //b
            1,                  //c
            0,                  //d
            0,                  //e
            0,                  //f
           -x2(0)*x1(0),        //g 
           -x2(0)*x1(1);        //h 
  }
  
  /** Return the 2D jacobian evaluation of a vector of parameters x, for the
   * 3 rows algebraic error model and with a set of static parameters x1, x2.
   */
  static void Jac(const Vec &x, const Vec3 &x1, const Vec3 &x2, Mat3X *jac) {
    // Errors on x
    *jac << 0,                //a
            0,                //b
            0,                //c
           -x1(0),            //d
           -x1(1),            //e
           -1,                //f
            x2(1)*x1(0),      //g 
            x2(1)*x1(1),      //h 
    // Errors on y    
            x1(0),            //a
            x1(1),            //b
            1,                //c
            0,                //d
            0,                //e
            0,                //f
           -x2(0)*x1(0),      //g 
           -x2(0)*x1(1),      //h 
    // Errors on z
           -x2(1)*x1(0),      //a 
           -x2(1)*x1(1),      //b
           -x2(1),            //c
            x2(0)*x1(0),      //d
            x2(0)*x1(1),      //e
            x2(0),            //f
            0,                //g 
            0;                //h 
  }
};
} // namespace homography2D
} // namespace homography
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_HOMOGRAPHY_JACOBIAN_H_

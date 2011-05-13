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

#ifndef LIBMV_MULTIVIEW_HOMOGRAPHY_OPTIMIZE_FUNCTOR_H_
#define LIBMV_MULTIVIEW_HOMOGRAPHY_OPTIMIZE_FUNCTOR_H_

#include "libmv/multiview/homography_jacobian.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace homography {
namespace homography2D {

/** A generic functor used by the nonlinear optimization algorithm to 
 * refine an homography 2D matrix.
 */
template <typename PARAMETERIZATION, 
          typename ERROR,
          typename JACOBIAN = Jacobian<PARAMETERIZATION, ERROR> >
class Analytic2RowsFunctor {
 public:
  typedef typename PARAMETERIZATION::Parameters::RealScalar Scalar;
  enum {
    InputsAtCompileTime = Eigen::Dynamic,
    ValuesAtCompileTime = Eigen::Dynamic
  };
  // TODO(julien) use PARAMETERIZATION::Parameters::RowsAtCompileTime instead
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;
     
  /// Function class constructor
  Analytic2RowsFunctor(const Mat3 &x1, const Mat3 &x2) : 
      inputs_(PARAMETERIZATION::Parameters::RowsAtCompileTime), 
      values_(2 * x1.cols()), x1_(x1), x2_(x2), residuals_(2, x1.cols()) {
    assert(x1.cols() == x2.cols());
    H_.setIdentity();
  }
    
  /// Function call to evaluate the cost function for a given set of parameters x
  int operator()(const Vec &x, Vec &fvec) {
    const int n = x1_.cols();
    //assert(x.size() == PARAMETERIZATION::Parameters::RowsAtCompileTime);
    assert(fvec.size() == 2 * n);
    PARAMETERIZATION::To(x, &H_);
    ERROR::Residuals(H_, x1_, x2_, &residuals_);
    fvec.segment(0, n) =  residuals_.row(0);
    fvec.segment(n, n) =  residuals_.row(1);
    return 0;
  }
  
  /// Function call to evaluate the jacobian of the cost function for a given 
  /// set of parameters x
  int df(const Vec &x, Mat &jac) {
    const int n = x1_.cols();
    //assert(x.size() == PARAMETERIZATION::Parameters::RowsAtCompileTime);
    //assert(jac.cols() == PARAMETERIZATION::Parameters::RowsAtCompileTime);
    assert(jac.rows() == 2 * n);
    Mat2X J(2, x.size());
    for(int i = 0; i < n; i++) {
      JACOBIAN::Jac(x, x1_.col(i), x2_.col(i), &J);
      jac.row(i) = J.row(0);
      jac.row(n + i) = J.row(1);
    }
    return 0;
  }
 public:
  int inputs() const { return inputs_; }
  int values() const { return values_; }
 protected:
  /// A set of 2D points (3xN matrix of column vectors).
  const Mat3X x1_;
  /// A set of 2D points (3xN matrix of column vectors).
  const Mat3X x2_; // TODO(julien) x2_ must be Mat3X to avoid conversions
  /// The Homography matrix
  Mat3 H_;
  /// The current 2D residuals
  Mat2X residuals_;
  /// Internal variables used by the optimization
  const int inputs_, values_;
};

} // namespace homography2D
} // namespace homography
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_HOMOGRAPHY_OPTIMIZE_FUNCTOR_H_

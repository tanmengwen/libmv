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
#include "libmv/multiview/structure_parameterization.h"

namespace libmv {
namespace point3D {

 /** 
 * A generic functor used by the nonlinear optimization algorithm to 
 * refine the projection matrix of an uncalibrated camera (affine/projective).
 */
template <typename PARAMETERIZATION, 
          typename ERROR,
          typename JACOBIAN = Jacobian2Rows<PARAMETERIZATION, ERROR> >
class Analytic2RowsFunctor {
 public:
  typedef PARAMETERIZATION::Parameters::Scalar Scalar;
  enum {
    InputsAtCompileTime = Eigen::Dynamic,
    ValuesAtCompileTime = Eigen::Dynamic
  };
  // TODO(julien) use PARAMETERIZATION::Parameters::RowsAtCompileTime instead
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;
     
  /// Function class constructor
  Analytic2RowsFunctor(const vector<Mat34> &Ps, const Mat &X, const Mat &x) : 
      inputs_(PARAMETERIZATION::Parameters::RowsAtCompileTime), 
      values_(2 * x.cols()), Q_(X), residuals_(2, x.cols()), P_(P) {
    assert(x.cols() == X.cols());
    if (x.rows() == 3)
      q_ = x;
    else
      EuclideanToHomogeneous(x, &q_);
    H_.setIdentity();
  }
    
  /// Function call to evaluate the cost function for a given set of parameters x
  int operator()(const Vec &x, Vec &fvec) {
    const int n = q_.cols();
    //assert(x.size() == PARAMETERIZATION::Parameters::RowsAtCompileTime);
    assert(fvec.size() == 2 * n);
    PARAMETERIZATION::To(x, &Q_);
    ERROR::Residuals(Ps_, Q_, q_, &residuals_);
    fvec.segment(0, n) =  residuals_.row(0);
    fvec.segment(n, n) =  residuals_.row(1);
    return 0;
  }
  
  /// Function call to evaluate the jacobian of the cost function for a given 
  /// set of parameters x
  int df(const Vec &x, Mat &jac) {
    const int n = q_.cols();
    //assert(x.size() == PARAMETERIZATION::Parameters::RowsAtCompileTime);
    //assert(jac.cols() == PARAMETERIZATION::Parameters::RowsAtCompileTime);
    assert(jac.rows() == 2 * n);
    // TODO(julien) avoid this copy?
    //PARAMETERIZATION::To(x, &Q_);
    for(int i = 0; i < n; i++) {
      JACOBIAN::Jacobian(Ps_[i],
                         PARAMETERIZATION::From(x, i), 
                         q_.col(i),
                         &J);
    }
    //std::cout << "jac : \n" <<jac<<"\n";
    return 0;
  }
 protected:
  int inputs() const { return m_inputs; }
  int values() const { return m_values; }
 protected:
  /// A set of 2D points (3xN matrix of column vectors).
  Mat3X q_;
  /// A set of 3D points (3xN or 4xN matrix of column vectors).
  Mat Q_;
  /// Camera projection matrices
  const vector<Mat34> &Ps_;
  /// The current 2D residuals
  Mat2X residuals_;
  /// Internal variables used by the optimization
  const int inputs_, values_;
};

}  // namespace point3D
}  // namespace libmv

#endif  // LIBMV_MULTIVIEW_STRUCTURE_OPTIMIZE_H_

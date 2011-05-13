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

#ifndef LIBMV_MULTIVIEW_STRUCTURE_PARAMETERIZATION_H_
#define LIBMV_MULTIVIEW_STRUCTURE_PARAMETERIZATION_H_

#include "libmv/numeric/numeric.h"

namespace libmv {
namespace point3D {
/** A parameterization of a set of 3D points structures that uses 3 
 * parameters (euclidean coordinates).
 */
template<typename T = double>
class EuclideanParameterization {
 public:
  typedef Eigen::Matrix<T, 3, Eigen::Dynamic> Parameters;     
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Parameterized;  

  /// Convert from a set of euclidean points to a vector
  static void To(const Parameters &p, Parameterized *q) { 
    q->resize(3 * p.template cols(), 1);
    for (int i = 0; i < p.cols(); ++i)
      q->template segment<3>(3 * i) << p.template col(i);
  }
  
  /// Convert from a vector of euclidean points to a set
  static void From(const Parameterized &q, Parameters *p) {
    const int n = q.rows() / 3;
    p->resize(3, n);
    for (int i = 0; i < n; ++i)
      p->template col(i) = q.template segment<3>(3 * i);
  }
  
  /// Return the vector of euclidean points from a parametrized set
  static Eigen::Matrix<T, 3, 1>& From(const Parameterized &q, int id) {
    return q.template segment<3>(3 * id);
  }
};

/** A parameterization of a set of 3D points structures that uses 4 
 * parameters (homogeneous coordinates).
 */
template<typename T = double>
class HomogeneousParameterization {
 public:
  typedef Eigen::Matrix<T, 4, Eigen::Dynamic> Parameters;  
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Parameterized; 

  /// Convert from a set of homogeneous points to a vector
  static void To(const Parameters &p, Parameterized *q) { 
    q->resize(4 * p.template cols(), 1);
    for (int i = 0; i < p.cols(); ++i)
      q->template segment<4>(4 * i) << p.template col(i);
  }

  /// Convert from a vector of homogeneous points to a set
  static void From(const Parameterized &q, Parameters *p) {
    const int n = q.rows() / 4;
    p->resize(4, n);
    for (int i = 0; i < n; ++i)
      p->template col(i) = q.template segment<4>(4 * i);
  }
  
  /// Return the vector of homogeneous points from a parametrized set
  static Eigen::Matrix<T, 4, 1>& From(const Parameterized &q, int id) {
    return q.template segment<4>(4 * id);
  }
};
} // namespace point3D
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_STRUCTURE_PARAMETERIZATION_H_

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

#include <Eigen/NonLinearOptimization>

#include "testing/testing.h"
#include "libmv/logging/logging.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/homography_optimize.h"

namespace {
using namespace libmv;
using namespace libmv::homography;

TEST(Homography2DAlgebraicError, HomographyGeneral) {
  Mat x1(3, 4);
  x1 <<  0, 1, 0, 5,
         0, 0, 2, 3,
         1, 1, 1, 1;
  Mat3 m;
  m <<   3, -1,  4,
         6, -2, -3,
         1, -3,  1;

  Mat x2(3, 4);
  for(int i = 0; i < x2.cols(); ++i) {
    x2.col(i) = m * x1.col(i);
    x2.col(i) /= x2(2, i);
  }
  std::cout << "q1\n" << x1 <<"\n";
  std::cout << "q2\n" << x2 <<"\n";

  Mat3 h_mat_init = m + Mat3::Random() * 1e-1;
  //h_mat_init(2, 2) = 1;
  std::cout << "Mat GT:\n" << m <<"\n";
  std::cout << "Mat Init:\n" << h_mat_init <<"\n";
  
  int info;
  Vec8 x8;
  Homography2DNormalizedParameterization<>::From(h_mat_init, &x8);
  // TODO(julien) avoid this copy!
  Vec x = x8;
  typedef homography2D::Analytic2RowsFunctor<Homography2DNormalizedParameterization<>, 
                                             homography2D::AlgebraicError> HomographyFunctor;
  HomographyFunctor homography_functor(x1, x2);
  Eigen::LevenbergMarquardt<HomographyFunctor> lm(homography_functor);
  //lm.parameters.maxfev = 400;
  info = lm.minimize(x);  
  
  Mat3 h_mat_final; 
  Homography2DNormalizedParameterization<>::To(x, &h_mat_final);
  EXPECT_TRUE(info > 0);
  
  std::cout << "Mat Final:\n" << h_mat_final <<"\n";
  std::cout << "info = " << info <<"\n";
  EXPECT_MATRIX_PROP(h_mat_final, m, 1e-6);
}

}

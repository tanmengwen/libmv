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

#include "libmv/logging/logging.h"
#include "libmv/multiview/homography_jacobian.h"
#include "libmv/numeric/numeric.h"
#include "testing/testing.h"

namespace {
using namespace libmv;
using namespace libmv::homography;

TEST(Homography2DAlgebraicErrJacobian2Rows, FuncApprox) {
  typedef homography2D::AlgebraicError Error;
  typedef Homography2DNormalizedParameterization<double> Param;
  typedef homography2D::Jacobian<Param, Error> HJac;  
  
  // Don't expect so much with the algebraic cost function
  const int kExpectedPrecision = 1e-2;

  Vec3 q1, q2; 
  Mat3 H;
  H << 1, 2, 3,
       2, 2, 4, 
       3, 5, 1;
  q1 << Vec2::Random()*5.0, 1;
  q2 = H * q1;
  q2 /= q2(2);
  // Add some real noise
  q2.head<2>() += Vec2::Random()*1e-3;  
  Vec8 x;
  Param::From(H, &x);
  Mat2X J(2, 8);
  // Estimates the jacobiam matrix
  HJac::Jac(x, q1, q2, &J);

  Vec8 dx;
  Mat3 H_dH;
  Vec3 Fx, Fx_dx;
  Vec2 FxJdx;
  for (int i = 0; i < 3; i++) {
    dx << Vec8::Random()*1e-6;
    Param::To(x + dx, &H_dH);
    Error::Residuals(H, q1, q2, &Fx);
    Fx /= Fx(2);
    Error::Residuals(H_dH, q1, q2, &Fx_dx);
    Fx_dx /= Fx_dx(2);  
    FxJdx = Fx.head<2>() + J * dx;
    // Check if  F(x + dx) ~ F(x) + J*dx
    EXPECT_MATRIX_NEAR(Fx_dx.head<2>(), FxJdx, kExpectedPrecision);
  }
}

TEST(Homography2DAlgebraicErrJacobian3Rows, FuncApprox) {
  typedef homography2D::AlgebraicError Error;
  typedef Homography2DNormalizedParameterization<> Param;
  typedef homography2D::Jacobian<Param, Error> HJac;  
  
  // Don't expect so much with the algebraic cost function
  const int kExpectedPrecision = 1e-2;

  Vec3 q1, q2; 
  Mat3 H;
  H << 1, 2, 3,
       2, 2, 4, 
       3, 5, 1;
  q1 << Vec2::Random()*5.0, 1;
  q2 = H * q1;
  q2 /= q2(2);
  // Add some real noise
  q2.head<2>() += Vec2::Random()*1e-3;  
  Vec8 x;
  Param::From(H, &x);
  Mat3X J(3, 8);
  // Estimates the jacobiam matrix
  HJac::Jac(x, q1, q2, &J);

  Vec8 dx;
  Mat3 H_dH;
  Vec3 Fx, Fx_dx;
  Vec3 FxJdx;
  for (int i = 0; i < 3; i++) {
    dx << Vec8::Random()*1e-6;
    Param::To(x + dx, &H_dH);
    Error::Residuals(H, q1, q2, &Fx);
    Fx /= Fx(2);
    Error::Residuals(H_dH, q1, q2, &Fx_dx);
    Fx_dx /= Fx_dx(2);  
    FxJdx = Fx + J * dx;
    // Check if  F(x + dx) ~ F(x) + J*dx
    EXPECT_MATRIX_NEAR(Fx_dx, FxJdx, kExpectedPrecision);
  }
}

} // namespace

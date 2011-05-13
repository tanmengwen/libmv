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
#include "libmv/multiview/homography_error.h"
#include "libmv/multiview/affine_jacobian.h"
#include "testing/testing.h"

namespace {
using namespace libmv::affine;
using namespace libmv::homography;

TEST(Affine2DAsymErrJacobian, FuncApprox) {
  typedef homography2D::AsymmetricError Error;
  typedef Affine2DGenericParameterization<> Param;

  typedef Jacobian<Param, Error> PointJac;  
  Vec3 q1, q2; q1 = Vec3::random()*5.0;
  Mat3 A;
  A << 1, 2, 3,
     0.5, 2, 4, 
       0, 0, 1;
  q2 = H * q1;
  // Add some real noise
  q2 += Vec3::random()*0.01;
  
  Vec8 x;
  Param::From(H, &x);
  Mat2X J(2, 8);
  PointJac::Jacobian(x, x1, x2, &J);
  
  Vec8 dx; dx << Vec8::random()*1e-3;
  dx /= dx(8);
  Mat3 H_dH;
  Param::To(x + dx, &H_dH);
  
  Vec3 Fx;
  Error::Residuals(H, q1, q2, &Fx);
  Fx /= Fx(2);
  Vec3 Fx_dx;
  Error::Residuals(H_dH, q1, q2, &Fx_dx);
  Fx_dx /= Fx_dx(2);
  
  // Check if  F(x + dx) ~ F(x) + J*dx
  EXPECT_MATRIX_NEAR(Fx_dx.topRows<2>(), Fx.topRows<2>() + J * dx, 1e-5);
}

} // namespace

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
#include "libmv/multiview/camera_error.h"
#include "libmv/multiview/structure_jacobian.h"
#include "testing/testing.h"

namespace {
using namespace libmv;
using namespace libmv::camera::pinhole;

TEST(HPoint3DAlgebraicErrJacobian, FuncApprox) {
  typedef AlgebraicError Error;
  typedef HomogeneousParameterization<> Param;

  typedef Jacobian2Rows<Param, Error> PointJac;  
  Vec4 Q; Q = Vec4::random()*5.0;
  Mat34 P;
  P << 1, 2, 3, 1,
       2, 3, 4, 0, 
       3, 4, 5, 1;
  Vec2 q;
  Project(P, Q, &q);
  // Add some real noise
  q += Vec2::random()*0.01;
  
  Vec4 x;
  Param::From(Q, &x);
  Mat2X J(2, 4);
  PointJac::Jacobian(Q, q, P, &J);
  
  Vec4 dx; dx << Vec4::random()*1e-3;
  dx /= dx(3);
  Vec3 Fx;
  Error::Residuals(P, x, q, &Fx);
  Fx /= Fx(2);
  Vec3 Fx_dx;
  Error::Residuals(P, x + dx, q, &Fx_dx);
  Fx_dx /= Fx_dx(2);
  
  // Check if  F(x + dx) ~ F(x) + J*dx
  EXPECT_MATRIX_NEAR(Fx_dx, Fx + J * dx, 1.5e-5);
}

} // namespace

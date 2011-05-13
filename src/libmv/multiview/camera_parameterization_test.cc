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
#include "libmv/multiview/camera_parameterization.h"
#include "testing/testing.h"

namespace {
using namespace libmv;


TEST(PerspCamFocalExtEulerParameterization, Roundtripping) {
  
  Mat34 P, P_roundtrip;
  Vec7 p; p << 1, 2, 3, 4, 5, 6, 7;
  Vec7 p_roundtrip;

  // Use the parameterization to get some arbitrary P matrix.
  // The initial p is ignored.
  PerspCamFocalExtEulerParameterization<>::To(p, &P);

  // Then go from the P matrix to p
  PerspCamFocalExtEulerParameterization<>::From(P, &p_roundtrip);

  // Now convert back to P from p 
  PerspCamFocalExtEulerParameterization<>::To(p_roundtrip, &P_roundtrip);

  // Check that going from P to p and back to H goes in a circle.
  EXPECT_MATRIX_PROP(P, P_roundtrip, 1.5e-8);
  
  /*
  Mat3 K;Mat4 C;
  Mat34 P = K * C;
  
  Vec x;
  PerspCamLiteIntExtEulerParameterization::To(K, C, &x);
  PerspCamLiteIntExtEulerParameterization::To(P, &x);
  Optimize(x, error)
  
  PerspCamLiteIntExtEulerParameterization::From(x, &K, &C);
  PerspCamLiteIntExtEulerParameterization::From(x, &P);*/
}

TEST(PerspCamLiteIntExtEulerParameterization, Roundtripping) {
  
  Mat34 P, P_roundtrip;
  Vec9 p; p << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Vec9 p_roundtrip;

  // Use the parameterization to get some arbitrary P matrix.
  // The initial p is ignored.
  PerspCamLiteIntExtEulerParameterization<>::To(p, &P);

  // Then go from the P matrix to p
  PerspCamLiteIntExtEulerParameterization<>::From(P, &p_roundtrip);

  // Now convert back to P from p 
  PerspCamLiteIntExtEulerParameterization<>::To(p_roundtrip, &P_roundtrip);

  // Check that going from P to p and back to H goes in a circle.
  EXPECT_MATRIX_PROP(P, P_roundtrip, 1.5e-8);
}

TEST(PerspCamFullIntExtEulerParameterization, Roundtripping) {
  
  Mat34 P, P_roundtrip;
  Vec11 p; p << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
  Vec11 p_roundtrip;

  // Use the parameterization to get some arbitrary P matrix.
  // The initial p is ignored.
  PerspCamFullIntExtEulerParameterization<>::To(p, &P);

  // Then go from the P matrix to p
  PerspCamFullIntExtEulerParameterization<>::From(P, &p_roundtrip);

  // Now convert back to P from p 
  PerspCamFullIntExtEulerParameterization<>::To(p_roundtrip, &P_roundtrip);

  // Check that going from P to p and back to H goes in a circle.
  EXPECT_MATRIX_PROP(P, P_roundtrip, 1.5e-8);
}

TEST(AffineCameraParameterization, Roundtripping) {
  Mat34 P, P_roundtrip;
  Vec8 p; p << 1, 2, 3, 4, 5, 6, 7, 8;
  Vec8 p_roundtrip;

  // Use the parameterization to get some arbitrary P matrix.
  // The initial p is ignored.
  AffineCameraParameterization<double>::To(p, &P);

  // Then go from the P matrix to p
  AffineCameraParameterization<double>::From(P, &p_roundtrip);

  // Now convert back to P from p 
  AffineCameraParameterization<double>::To(p_roundtrip, &P_roundtrip);

  // Check that going from P to p and back to H goes in a circle.
  EXPECT_MATRIX_PROP(P, P_roundtrip, 1.5e-8);
}

TEST(ProjectiveCameraParameterization, Roundtripping) {
  Mat34 P, P_roundtrip;
  Vec11 p; p << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
  Vec11 p_roundtrip;

  // Use the parameterization to get some arbitrary P matrix.
  // The initial p is ignored.
  ProjectiveCameraParameterization<double>::To(p, &P);

  // Then go from the P matrix to p
  ProjectiveCameraParameterization<double>::From(P, &p_roundtrip);

  // Now convert back to P from p 
  ProjectiveCameraParameterization<double>::To(p_roundtrip, &P_roundtrip);

  // Check that going from P to p and back to H goes in a circle.
  EXPECT_MATRIX_PROP(P, P_roundtrip, 1.5e-8);
}

} // namespace

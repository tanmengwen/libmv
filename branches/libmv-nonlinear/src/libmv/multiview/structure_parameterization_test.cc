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
#include "libmv/multiview/structure_parameterization.h"
#include "testing/testing.h"

namespace {
using namespace libmv;

TEST(Point3DEuclideanParameterization, Roundtripping) {
  Vec4 q, q_roundtrip;
  Vec3 p; p << 1, 2, 3;
  Vec3 p_roundtrip;
  Point3DEuclideanParameterization<>::To(p, &q);
  Point3DEuclideanParameterization<>::From(q, &p_roundtrip);
  Point3DEuclideanParameterization<>::To(p_roundtrip, &q_roundtrip);
  EXPECT_MATRIX_PROP(q, q_roundtrip, 1.5e-8);
}

TEST(Point3DHomogeneousParameterization, Roundtripping) {
  Vec4 q, q_roundtrip;
  Vec4 p; p << 1, 2, 3, 4;
  Vec4 p_roundtrip;

  Point3DHomogeneousParameterization<>::To(p, &q);
  Point3DHomogeneousParameterization<>::From(q, &p_roundtrip);
  Point3DHomogeneousParameterization<>::To(p_roundtrip, &q_roundtrip);
  EXPECT_MATRIX_PROP(q, q_roundtrip, 1.5e-8);
}

} // namespace

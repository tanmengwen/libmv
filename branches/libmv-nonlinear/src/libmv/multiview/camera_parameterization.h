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

#ifndef LIBMV_MULTIVIEW_CAMERA_PARAMETERIZATION_H_
#define LIBMV_MULTIVIEW_CAMERA_PARAMETERIZATION_H_

#include "libmv/multiview/euclidean_parameterization.h"
#include "libmv/multiview/projection.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace camera {
namespace pinhole {
namespace calibrated {

/** A parameterization of a perspective camera that uses 7 parameters that 
 * includes the focal length, and extrinsics parameters: orientation 
 * (Euler angles) and position.
 * 
 * The angle convention can be chosen with the template arguments N0, N1 and N2 
 * with a possible value of 0 (X), 1 (Y) and 2 (Z). 
 * The default convention is Y-X-Z.
 * 
 * The projection matrix P is built from a list of 11 parameters 
 * (f, e0, e1, e2, tx, ty, tz) as follows:  
 * 
 *  P = K * C,
 * 
 * where the euclidean pose matrix C is built from the last 6 parameters 
 * (e0, e1, e2, tx, ty, tz) as follows
 * 
 *      |      tx|
 *  C = | R(e) ty|
 *      |      tz|
 * 
 * with R(e) = Rot(Y, e0) * Rot(X, e1) * Rot(Z, e2) for N0 = 1, N1 = 0, N2 = 2.
 * The calibration matrix K is built as follows
 * 
 *      |f 0 0|
 *  K = |0 f 0|
 *      |0 0 1|
 */
template<typename T = double, 
         int N0 = 1, 
         int N1 = 0, 
         int N2 = 2>
class PerspCamFocalExtEulerParameterization {
 public:
  typedef Eigen::Matrix<T, 7, 1> Parameters; // f, e0, e1, e2, tx, ty, tz
  typedef Eigen::Matrix<T, 3, 4> ParameterizedProjection;  // P = K * C
  typedef Eigen::Matrix<T, 3, 3> ParameterizedIntrinsic;   // K
  typedef Eigen::Matrix<T, 4, 4> ParameterizedExtrinsic;   // C = [R,t]

  /// Convert from the 7 parameters (f, e0, e1, e2, tx, ty, tz) 
  /// to a projection matrix P
  static void To(const Parameters &p, ParameterizedProjection *P) { 
    ParameterizedIntrinsic K;
    ParameterizedExtrinsic C;
    To(p, &K, &C);
    *P = K * C.template block<3, 4>(0, 0);
  }
  
  /// Convert from the 7 parameters (f, e0, e1, e2, tx, ty, tz) 
  /// to intrinsic and extrinsic matrices (K, C)
  /// \note Only parametrized elements are updated in K. 
  static void To(const Parameters &p, ParameterizedIntrinsic *K, 
                                      ParameterizedExtrinsic *C) { 
    (*K)(0,0) =  p(0);
    (*K)(1,1) =  p(0);
    // TODO(julien) find a better solution?
    /**K << p(0),   0,  0,
            0,  p(0), 0,
            0,    0,  1;*/
    Euclidean3DEulerParameterization<T, N0, N1, N2>::To(
        p.template segment<6>(1), C);
  }

  /// Convert from a projection matrix P to the 7 parameters 
  /// (f, e0, e1, e2, tx, ty, tz).
  static void From(const ParameterizedProjection &P, Parameters *p) {
    ParameterizedIntrinsic K;
    ParameterizedExtrinsic C;
    Mat3 R; Vec3 t;
    KRt_From_P(P, &K, &R, &t);
    C.template block<3,3>(0,0) = R;
    C.col(3).template head<3>() = t;
    C.row(3) << 0.0, 0.0, 0.0, 1.0;
    From(K, C, p);
  }
  
  /// Convert from the intrinsic and extrinsic matrices (K, C) 
  /// to the 7 parameters (f, e0, e1, e2, tx, ty, tz) 
  static void From(const ParameterizedIntrinsic &K, 
                   const ParameterizedExtrinsic &C,
                   Parameters *p) { 
    Eigen::Matrix<T, 6, 1> v;
    Euclidean3DEulerParameterization<T, N0, N1, N2>::From(C, &v);
    *p << K(0,0), v;
  }
};

/** A parameterization of a perspective camera that uses 9 parameters that 
 * includes intrinsics: focal length and principal point, 
 * and extrinsics: orientation (Euler angles) and position.
 * 
 * The angle convention can be chosen with the template arguments N0, N1 and N2 
 * with a possible value of 0 (X), 1 (Y) and 2 (Z). 
 * The default convention is Y-X-Z.
 * 
 * The projection matrix P is built from a list of 11 parameters 
 * (f, cu, cv, e0, e1, e2, tx, ty, tz) as follows:  
 * 
 *  P = K * C,
 * 
 * where the euclidean pose matrix C is built from the last 6 parameters 
 * (e0, e1, e2, tx, ty, tz) as follows
 * 
 *      |      tx|
 *  C = | R(e) ty|
 *      |      tz|
 * 
 * with R(e) = Rot(Y, e0) * Rot(X, e1) * Rot(Z, e2) for N0 = 1, N1 = 0, N2 = 2.
 * The calibration matrix K is built as follows
 * 
 *      |f 0 cu|
 *  K = |0 f cv|
 *      |0 0  1|
 */
template<typename T = double, 
         int N0 = 1, 
         int N1 = 0, 
         int N2 = 2>
class PerspCamLiteIntExtEulerParameterization {
 public:
  typedef Eigen::Matrix<T, 9, 1> Parameters; // f, cu, cv, e0, e1, e2, tx, ty, tz
  typedef Eigen::Matrix<T, 3, 4> ParameterizedProjection;  // P = K * C
  typedef Eigen::Matrix<T, 3, 3> ParameterizedIntrinsic;   // K
  typedef Eigen::Matrix<T, 4, 4> ParameterizedExtrinsic;   // C = [R,t]

  /// Convert from the 9 parameters (f, cu, cv, e0, e1, e2, tx, ty, tz) 
  /// to a projection matrix P
  static void To(const Parameters &p, ParameterizedProjection *P) { 
    ParameterizedIntrinsic K;
    ParameterizedExtrinsic C;
    To(p, &K, &C);
    *P = K * C.template block<3, 4>(0, 0);
  }
  
  /// Convert from the 9 parameters (f, cu, cv, e0, e1, e2, tx, ty, tz) 
  /// to intrinsic and extrinsic matrices (K, C)
  /// \note Only parametrized elements are updated in K. 
  static void To(const Parameters &p, ParameterizedIntrinsic *K, 
                                      ParameterizedExtrinsic *C) { 
    (*K)(0,0) =  p(0);
    (*K)(1,1) =  p(0);
    (*K)(0,2) =  p(1);
    (*K)(1,2) =  p(2);
    // TODO(julien) find a better solution?
    /**K << p(0),   0,  p(1),
            0,  p(0), p(2),
            0,    0,    1;*/
    Euclidean3DEulerParameterization<T, N0, N1, N2>::To(
        p.template segment<6>(3), C);
  }

  /// Convert from a projection matrix P to the 9 parameters 
  /// (f, cu, cv, e0, e1, e2, tx, ty, tz).
  static void From(const ParameterizedProjection &P, Parameters *p) {
    ParameterizedIntrinsic K;
    ParameterizedExtrinsic C;
    Mat3 R; Vec3 t;
    KRt_From_P(P, &K, &R, &t);
    C.template block<3,3>(0,0) = R;
    C.col(3).template head<3>() = t;
    C.row(3) << 0.0, 0.0, 0.0, 1.0;
    From(K, C, p);
  }
  
  /// Convert from the intrinsic and extrinsic matrices (K, C) 
  /// to the 9 parameters (f, cu, cv, e0, e1, e2, tx, ty, tz) 
  static void From(const ParameterizedIntrinsic &K, 
                   const ParameterizedExtrinsic &C,
                   Parameters *p) { 
    Eigen::Matrix<T, 6, 1> v;
    Euclidean3DEulerParameterization<T, N0, N1, N2>::From(C, &v);
    *p << K(0,0), K(0,2), K(1,2), v;
  }
};

/** A parameterization of a perspective camera that uses 11 parameters that 
 * includes intrinsics: focal lengths, principal point, skew factor, 
 * and extrinsics: orientation (Euler angles) and position.
 * 
 * The angle convention can be chosen with the template arguments N0, N1 and N2 
 * with a possible value of 0 (X), 1 (Y) and 2 (Z). 
 * The default convention is Y-X-Z.
 * 
 * The projection matrix P is built from a list of 11 parameters 
 * (fx, fy, cu, cv, s, e0, e1, e2, tx, ty, tz) as follows:  
 * 
 *  P = K * C,
 * 
 * where the euclidean pose matrix C is built from the last 6 parameters 
 * (e0, e1, e2, tx, ty, tz) as follows
 * 
 *      |      tx|
 *  C = | R(e) ty|
 *      |      tz|
 * 
 * with R(e) = Rot(Y, e0) * Rot(X, e1) * Rot(Z, e2) for N0 = 1, N1 = 0, N2 = 2.
 * The calibration matrix K is built as follows
 * 
 *      | fx  s  cu|
 *  K = |  0  fy cv|
 *      |  0  0   1|
 */
template<typename T = double, 
         int N0 = 1, 
         int N1 = 0, 
         int N2 = 2>
class PerspCamFullIntExtEulerParameterization {
 public:
  typedef Eigen::Matrix<T,11, 1> Parameters; // fx, fy, cu, cv, s, e0, e1, e2, tx, ty, tz
  typedef Eigen::Matrix<T, 3, 4> ParameterizedProjection;  // P = K * C
  typedef Eigen::Matrix<T, 3, 3> ParameterizedIntrinsic;   // K
  typedef Eigen::Matrix<T, 4, 4> ParameterizedExtrinsic;   // C = [R,t]

  /// Convert from the 11 parameters (fx, fy, cu, cv, s, e0, e1, e2, tx, ty, tz) 
  /// to a perspective projection matrix
  static void To(const Parameters &p, ParameterizedProjection *P) { 
    ParameterizedIntrinsic K;
    ParameterizedExtrinsic C;
    To(p, &K, &C);
    *P = K * C.template block<3, 4>(0, 0);
  }

  /// Convert from the 11 parameters (fx, fy, cu, cv, s, e0, e1, e2, tx, ty, tz)  
  /// to intrinsic and extrinsic matrices (K, C)
  static void To(const Parameters &p, ParameterizedIntrinsic *K, 
                                      ParameterizedExtrinsic *C) { 
    *K << p(0), p(4), p(2),
            0,  p(1), p(3),
            0,    0,    1;
    Euclidean3DEulerParameterization<T, N0, N1, N2>::To(
        p.template segment<6>(5), C);
  }
  
  /// Convert from a perspective projection matrix to the 11 parameters 
  /// (f, cu, cv, e0, e1, e2, tx, ty, tz) .
  static void From(const ParameterizedProjection &P, Parameters *p) {
    ParameterizedIntrinsic K;
    ParameterizedExtrinsic C;
    Mat3 R; Vec3 t;
    KRt_From_P(P, &K, &R, &t);
    C.template block<3,3>(0,0) = R;
    C.col(3).template head<3>() = t;
    C.row(3) << 0.0, 0.0, 0.0, 1.0;
    From(K, C, p);
  }
  /// Convert from the intrinsic and extrinsic matrices (K, C) 
  /// to the 11 parameters (fx, fy, cu, cv, s, e0, e1, e2, tx, ty, tz)  
  static void From(const ParameterizedIntrinsic &K, 
                   const ParameterizedExtrinsic &C,
                   Parameters *p) { 
    Eigen::Matrix<T, 6, 1> v;
    Euclidean3DEulerParameterization<T, N0, N1, N2>::From(C, &v);
    *p << K(0,0), K(1,1), K(0,2), K(1,2), K(0,1), v;
  }
};

} // namespace calibrated

namespace uncalibrated {
/** A parameterization of an affine camera that uses 8 parameters so 
 * that the last line of the projection matrix is (0, 0, 0, 1).
 * The projection matrix P is built from a list of 8 parameters (a, b,...g, h)
 * as follows
 *          |a b c d| 
 *      P = |e f g h|
 *          |0 0 0 1| 
 */
template<typename T = double>
class AffineCameraParameterization {
 public:
  typedef Eigen::Matrix<T, 8, 1> Parameters;     // a, b, ... g, h
  typedef Eigen::Matrix<T, 3, 4> Parameterized;  // P

  /// Convert from the 8 parameters (a, b, ... g, h) 
  /// to an affine projection matrix
  static void To(const Parameters &p, Parameterized *P) { 
    *P << p(0), p(1), p(2), p(3),
          p(4), p(5), p(6), p(7),
          0.0,  0.0,  0.0,  1.0;    
  }

  /// Convert from an affine projection matrix to its 8 parameters.
  static void From(const Parameterized &P, Parameters *p) {
    *p << P(0, 0), P(0, 1), P(0, 2), P(0, 3),
          P(1, 0), P(1, 1), P(1, 2), P(1, 3);
  }
};  

/** A parameterization of a projective camera that uses 11 parameters so 
 * that the projection matrix is normalized (H(2,3) == 1).
 * The projection matrix P is built from a list of 11 parameters (a, b,...j, k)
 * as follows
 *          |a b c d| 
 *      P = |e f g h|
 *          |i j k 1| 
 */
template<typename T = double>
class ProjectiveCameraParameterization {
 public:
  typedef Eigen::Matrix<T, 11, 1> Parameters;    // a, b, ... j, k
  typedef Eigen::Matrix<T, 3, 4> Parameterized;  // P

  /// Convert from the 11 parameters (a, b, ... j, k) 
  /// to a projective projection matrix
  static void To(const Parameters &p, Parameterized *P) { 
    *P << p(0), p(1), p(2),  p(3),
          p(4), p(5), p(6),  p(7),
          p(8), p(9), p(10), 1.0;    
  }

  /// Convert from a projective projection matrix to its 11 parameters.
  static void From(const Parameterized &P, Parameters *p) {
    *p << P(0, 0), P(0, 1), P(0, 2), P(0, 3),
          P(1, 0), P(1, 1), P(1, 2), P(1, 3),
          P(2, 0), P(2, 1), P(2, 2);
  }
};
} // namespace uncalibrated
} // namespace pinhole
} // namespace camera
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_CAMERA_PARAMETERIZATION_H_

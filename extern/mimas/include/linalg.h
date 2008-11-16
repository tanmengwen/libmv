#ifndef LINALG_H
#define LINALG_H

#include <boost/numeric/ublas/banded.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <cassert>
//#include "mimasconfig.h"
#include "angle.h"
#include "mimasexception.h"


#define HAVE_LIBLAPACK

namespace mimas {

/** @defgroup linearAlgebra Linear Algebra
    mimas offers support for linear algebra using
    <A HREF="http://www.boost.org/libs/numeric/ublas/doc/index.htm">boost's
    uBLAS</A>-library and the
    <A HREF="http://www.netlib.org/lapack/lug/lapack_lug.html">LAPACK</A>-library.
    @{ */
/** @defgroup ublas Mimas linear algebra operations.
    This group contains Mimas operations, for vectors and matrices. All
    functionality provided by \c mimas::vector formerly, should be provided
    here.
    @author Stuart Meikle (stu@stumeikle.org)
    @date Sat Mar  1 2003 BST
    @{ */
/** Compute unit vector.
    Compute unit vector \f$\frac{\vec{x}}{|\vec{x}|}\f$.
    @param x input vector (non-zero).
    @return Vector scaled to length 1. */
template< typename T >
boost::numeric::ublas::vector< T > unit
   ( const boost::numeric::ublas::vector< T > &x )
{
  assert( boost::numeric::ublas::norm_2( x ) > 0 );
  return x / boost::numeric::ublas::norm_2( x );
}

/** Scalar cross product.
    The scalar cross product for 2-dimensional vectors is
    \f$a\times b:=a_1\,b_2-a_2\,b_1\f$. It is rotational invariant.

    For scalar product see \c boost::inner_prod.
    @param a first vector.
    @param b second vector. */
template< typename T >
T scalar_cross_product( const boost::numeric::ublas::vector< T > &a, 
                       const boost::numeric::ublas::vector< T > &b )
{
  assert( a.size() == 2 );
  assert( b.size() == 2 );
  return a(0) * b(1) - a(1) * b(0);
}

/** Compute angle between two 2-dimensional vectors.
    @param a first vector.
    @param b second vector.
    @return angle to rotate \c a onto \c b */
template< typename T >
angle getAngle( const boost::numeric::ublas::vector< T > &a, 
                const boost::numeric::ublas::vector< T > &b )
{
  return angle( scalar_cross_product< T >( a, b ), inner_prod( a, b ) );
}

/** 2-dimensional rotation in the mathematical direction.
    Attention! The direction of rotation is oposite to the one of
    \c vector::rotate. */
template< typename T >
boost::numeric::ublas::vector< T > rotate
   ( const boost::numeric::ublas::vector< T > &v, const angle &a );

/** Cross product for 3-dimensional vectors.
    Compute
    \f$a\times b:=(a_2\,b_3-a_3\,b_2,a_3\,b_1-a_1\,b_3,
    a_1\,b_2-a_2\,b_1)^\top\f$
    @param a first vector.
    @param b second vector.
    @return cross product of \c a and \c b. */
template< typename T >
boost::numeric::ublas::vector< T > crossProduct
   ( boost::numeric::ublas::vector< T > &a,
     boost::numeric::ublas::vector< T > &b );

/** Rodrigues' rotation formula

    Computes the rotation of a point around an axis of given vector,
    and passing through the origin.

    @param u the axis of the rotation (must be a unit 3-D vector) 
    @param v the vector to rotate 
    @param angle the angle of the rotation
    @author Julien Faucher (faucherj@gmail.com)
 */
template< typename T >
boost::numeric::ublas::vector< T > rodrigues
   ( boost::numeric::ublas::vector< T > const &u,
     boost::numeric::ublas::vector< T > const &v,
     double angle);

/** Determinant of a matrix using the LU factorization.

    The decomposition used is the one provided by uBLAS.

    @param M the matrix to compute the determinant of.
    @author Julien Faucher (faucherj@gmail.com)
 */
template< typename T >
double determinant (boost::numeric::ublas::matrix< T > const &M);

///@}

#ifdef HAVE_LIBLAPACK

/** @defgroup lapack Lapack functions
    Wrappers or bindings for Lapack functions are provided. The matrix- and
    vector-classes of boost are used as datatypes.

    Note, that lapack uses matrices in <B>column-major</B> orientation. You
    are forced to convert to
    \code
    boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major >
    \endcode
    The default orientation of boost matrices is row-major.

    @author Bala Amavasai (bala@amavasai.org)
    @author Jan Wedekind (jan@wedesoft.de)
    @todo Extend the solver-classes to complex values.
    @date Fri May 31 12:07:52 UTC 2002
    @{ */
/** Function-object for gglse.
    @see gglse */
template< typename T >
struct gglse_t
{
  ///
  typedef boost::numeric::ublas::vector< T > Vector;
  ///
  typedef boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major > Matrix;
  /// Invoke lapack method.
  Vector operator()( const Matrix &_A, const Matrix &_B,
                     const Vector &_c, const Vector &_d ) const;
};

/** Solve the linear equality-constrained least squares (LSE) problem.
    \f$A\f$, \f$\vec{c}\f$, \f$B\f$ and \f$\vec{d}\f$ are given and
    \f$\vec{x}\f$ is
    sought:
    
    minimize \f$\vec{x}:=\displaystyle\mathop{argmin}_{\vec{x}}|\vec{c}-A\,\vec{x}|\f$
    subject to \f$B\,\vec{x}=\vec{d}\f$.

    See manpage <A HREF="man:/dgglse">dgglse(1)</A> for more documentation.
    @see gglse_t */
template< typename T >
boost::numeric::ublas::vector< T > gglse
  ( const boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > &A,
    const boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > &B,
    const boost::numeric::ublas::vector< T > &c,
    const boost::numeric::ublas::vector< T > &d )
{ return gglse_t< T >()( A, B, c, d ); }

/** Function-object for inv.
    @see inv */
template< typename T >
struct inv_t
{
  typedef boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major > Matrix;
  /// Invoke lapack method.
  Matrix operator()( const Matrix &A ) const;
};

/** Compute matrix inverse by using LU factorization.
    The matrix \f$A\in\mathbf{K}^{n\times n}\f$ is given and the inverse
    \f$A^{-1}\f$ has to be computed.

    See manpages <A HREF="man:/dgetrf">dgetrf(1)</A> and
    <A HREF="man:/dgetri">dgetri(1)</A> for more documentation.
    @see inv_t */
template< typename T >
boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major > inv
  ( const boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major > &A )
{ return inv_t< T >()( A ); }

/** Function-object for gels.
    @see gels */
template< typename T >
struct gels_t
{
  ///
  typedef boost::numeric::ublas::vector< T > Vector;
  ///
  typedef boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major > Matrix;
  /// Invoke lapack method.
  Vector operator()( const Matrix &_A, const Vector &_b ) const;
};

/** solve overdetermined linear systems using QR factorization.
    Solve the minimum least square problem. \f$A\f$ and \f$\vec{b}\f$ are given
    and \f$\vec{x}\f$ is the desired solution for
    \f[\vec{x}:=\displaystyle\mathop{argmin}_{\vec{x}}|\vec{b}-A\,\vec{x}|\f]

    This is much faster than computing
    \f$\vec{x}:=(A^\top\,A)^{-1}\,A^\top\,\vec{b}\f$ with the matrix-classes.

    See manpage <A HREF="man:/dgels">dgels(1)</A> for more documentation. The
    parametrisation for underdetermined systems wasn't working as expected and
    was not included in the interface therefore.
    @see gels_t */
template< typename T >
boost::numeric::ublas::vector< T > gels
  ( const boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > &A,
    const boost::numeric::ublas::vector< T > &b )
{ return gels_t< T >()( A, b ); }

/** Function-object for gesvd.
    @see gesvd */
template< typename T >
struct gesvd_t
{
  ///
  typedef boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major > Matrix;
  ///
  typedef boost::numeric::ublas::banded_matrix< T > BandedMatrix;
  /// Invoke lapack-method.
  BandedMatrix operator()( const Matrix &_A, Matrix *_U, Matrix *_Vt )
       throw(mimasexception);
};

/** Compute the singular value decomposition (SVD) of a matrix A.
    \f$A\f$ is given. \f$U\f$, \f$\Sigma\f$ and \f$V^\top\f$ have to be
    determined, such that:
    \f$A=U\,\Sigma\,V^\top\f$ and
    \f$\Sigma=\mathop{diag}(\sigma_1,\sigma_2,\ldots,\sigma_n)\f$ diagonal
    matrix with the singular values \f$\sigma_1,\sigma_2,\ldots,\sigma_n\f$
    (\f$\sigma_i\ge 0\f$) in descending order. The singular values are sorted
    in descending order.

    Note that the singular value decomposition doesn't necessarily converge.
    In the later case an exception is thrown.

    See \c mimas::gesdd, which is a faster algorithm.

    See manpage <A HREF="man:/dgesvd">dgesvd(1)</A> for more documentation.

    @param A Matrix to be decomposed.
    @param U Returns matrix with left hand singular vectors \f$U\f$ (may be
    \c NULL, if it is of no interest).
    @param Vt Returns transposed matrix with right hand singular vectors
    \f$V^\top\f$ (may be \c NULL, if it is of no interest).
    @return Diagonal matrix \f$\Sigma\f$.

    @todo Does not converge on AMD64-processor.
    @see gesdd
    @see gesvd_t */
template< typename T >
boost::numeric::ublas::banded_matrix< T > gesvd
  ( const boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > &A,
    boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > *U,
    boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > *Vt )
{ return gesvd_t< T >()( A, U, Vt ); }

/** Function-wrapper for gesdd.
    @see gesdd */
template< typename T >
struct gesdd_t
{
  ///
  typedef boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major > Matrix;
  ///
  typedef boost::numeric::ublas::banded_matrix< T > BandedMatrix;
  /// Invoke lapack-method.
  BandedMatrix operator()( const Matrix &_A, Matrix *_U, Matrix *_Vt )
       throw (mimasexception);
};

/** Compute the singular value decomposition (SVD) of a matrix A (fast).
    Same as \c mimas::gesvd but faster algorithm (divide-and-conquer
    approach).

    Note that the singular value decomposition doesn't necessarily converge.
    In the later case an exception is thrown.

    See manpage <A HREF="man:/dgesdd">dgesdd(1)</A> for more documentation.

    Note, that, in contrast to the interface of \c mimas::gesvd, \c U
    and \c Vt must both be defined or both be \c NULL!
    @param A Matrix to be decomposed.
    @param U Returns matrix with left hand singular vectors \f$U\f$ (may be
    \c NULL, if it is of no interest).
    @param Vt Returns transposed matrix with right hand singular vectors
    `\f$V^\top\f$ (may be \c NULL, if it is of no interest).
    @return Diagonal matrix \f$\Sigma\f$.
    @see gesvd
    @see gesdd_t */
template< typename T >
boost::numeric::ublas::banded_matrix< T > gesdd
  ( const boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > &A,
    boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > *U,
    boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > *Vt )
{ return gesdd_t< T >()( A, U, Vt ); }

/** Function-wrapper for syev.
    @see syev */
template< typename T >
struct syev_t
{
  ///
  typedef boost::numeric::ublas::matrix< T, boost::numeric::ublas::column_major > Matrix;
  ///
  typedef boost::numeric::ublas::symmetric_matrix< T, boost::numeric::ublas::upper > SymmetricMatrix;
  ///
  typedef boost::numeric::ublas::diagonal_matrix< T > EigenValues;
  /// Invoke lapack-method.
  EigenValues operator()( const SymmetricMatrix &_A, Matrix *_E ) const
    throw (mimasexception);
};

/** Compute all eigenvalues and, optionally, eigenvectors of a real symmetric matrix A.
    The eigenvalues and eigenvectors of a symmetric matrix
    \f$A\in\mathbf{K}^{n\times n}\f$ can be computed.
    
    The eigenvectors are orthogonal by nature and normalised by definition.
    If the matrix \f$E\f$ contains the eigenvectors of \f$A\f$, then the
    eigentransform can be written as (\f$E\f$ is orthonormal \f$\Rightarrow\f$
    \f$E^{-1}=E^\top\f$):
    \f[A=E\,\Lambda\,E^\top\f]

    Where \f$\Lambda=\mathop{diag}(\lambda_1,\lambda_2,\ldots,\lambda_n)\f$ is
    a diagonal matrix with the eigenvalues
    \f$\lambda_1,\lambda_2,\ldots,\lambda_n\f$ in ascending order.

    See manpage <A HREF="man:/dsyev">dsyev(1)</A> for more documentation.

    Compute eigenvalues and, optionally, the eigenvectors of A.
    @param A Symmetric matrix (with upper storage) to compute
    eigentransform for.
    @param E Pointer to matrix, where the eigenvectors have to be written
    to (may be \c NULL, if it is of no interest).
    @return Diagonal matrix with eigenvalues in ascending order.
    @see syev_t */
template< typename T >
boost::numeric::ublas::diagonal_matrix< T > syev
  ( const boost::numeric::ublas::symmetric_matrix
      < T, boost::numeric::ublas::upper > &A,
    boost::numeric::ublas::matrix
      < T, boost::numeric::ublas::column_major > *E )
{ return syev_t< T >()( A, E ); }


/** Function-object for pptrf.
    @see pptrf */
template< typename T >
struct pptrf_t
{
  ///
  typedef boost::numeric::ublas::triangular_matrix 
    < T, boost::numeric::ublas::upper, boost::numeric::ublas::column_major > TriangularMatrix;
  ///
  typedef boost::numeric::ublas::symmetric_matrix 
    < T, boost::numeric::ublas::upper, boost::numeric::ublas::column_major > SymmetricMatrix;

  /// Invoke lapack-method.
  TriangularMatrix operator()( const SymmetricMatrix &_A )
       throw(mimasexception);
};

/** Compute the Cholesky decomposition of a symmetric matrix A.

    Given a symmetric positive-definite matrix\f$A\in\mathbf{K}^{n\times n}\f$,
    the Cholesky decomposition returns the upper triangular matrix
    \f$U\in\mathbf{K}^{n\times n}\f$ such that:
    \f[A=U^\top\,U\f]

    @param A Symmetric matrix (with upper and column-based storage) to compute
    Cholesky decomposition for.
    @return Upper triangular matrix (with column-based storage).
    @author Julien Faucher (faucherj@gmail.com)
    @see pptrf_t */
template< typename T >
boost::numeric::ublas::triangular_matrix 
 < T, boost::numeric::ublas::upper, boost::numeric::ublas::column_major  > pptrf
  ( const boost::numeric::ublas::symmetric_matrix
      < T, boost::numeric::ublas::upper, boost::numeric::ublas::column_major  > &A)
{ return pptrf_t< T >()( A ); }

///@}

#endif

///@}

};

#include "linalg.tcc"

#endif


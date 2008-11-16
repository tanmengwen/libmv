#ifndef LINALG_TCC
#define LINALG_TCC

#include <algorithm>
#ifdef HAVE_LIBLAPACK
#include <boost/shared_ptr.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <cmath>
extern "C" {
#include <f2c.h>
#include "clapack.h"
}

// Problem with f2c defining macros.
#ifdef abs
#undef abs
#endif
#ifdef dabs
#undef dabs
#endif
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#ifdef dmin
#undef dmin
#endif
#ifdef dmax
#undef dmax
#endif
#ifdef bit_test
#undef bit_test
#endif
#ifdef bit_clear
#undef bit_clear
#endif
#ifdef bit_set
#undef bit_set
#endif

#endif

namespace mimas {

template< typename T >
boost::numeric::ublas::vector< T > rotate
   ( const boost::numeric::ublas::vector< T > &v, const angle &a )
{
  assert( v.size() == 2 );

  double
   cosa = cos( a ),
   sina = sin( a );
  
  boost::numeric::ublas::vector< T > retVal( 2 );

  //type casting needed to avoid warning message 
  //if you are using vector<int> for instance
  //This might necessary in following functions. 
  retVal(0) = T(v(0) * cosa - v(1)*sina);
  retVal(1) = T(v(0) * sina + v(1)*cosa);
  
  return retVal;
}

template< typename T >
boost::numeric::ublas::vector< T > crossProduct
   ( boost::numeric::ublas::vector< T > &a,
     boost::numeric::ublas::vector< T > &b )
{
   assert( a.size() == 3 && b.size() == 3 );
   boost::numeric::ublas::vector< T > retVal( 3 );
   retVal(0) = a(1) * b(2) - a(2) * b(1);
   retVal(1) = a(2) * b(0) - a(0) * b(2);
   retVal(2) = a(0) * b(1) - a(1) * b(0);
   return retVal;
}

template< typename T >
boost::numeric::ublas::vector< T > rodrigues
   ( boost::numeric::ublas::vector< T > const &u,
     boost::numeric::ublas::vector< T > const &v,
     double angle)
{
  boost::numeric::ublas::matrix< T > m(3,3);
  boost::numeric::ublas::vector< T > w;

  m(0,0) = cos(angle) + u(0) * u(0) * (1 - cos(angle));
  m(0,1) = u(0) * u(1) * (1 - cos(angle)) - u(2) * sin(angle);
  m(0,2) = u(1) * sin(angle) + u(0) * u(2) * (1 - cos(angle));
  m(1,0) = u(2) * sin(angle) + u(0) * u(1) * (1 - cos(angle)); 
  m(1,1) = cos(angle) + u(1) * u(1) * (1 - cos(angle));
  m(1,2) = - u(0) * sin(angle) + u(1) * u(2) * (1 - cos(angle)); 
  m(2,0) = - u(1) * sin(angle) + u(0) * u(2) * (1 - cos(angle)); 
  m(2,1) = u(0) * sin(angle) + u(1) * u(2) * (1 - cos(angle)); 
  m(2,2) = cos(angle) + u(2) * u(2) * (1 - cos(angle));

  w = boost::numeric::ublas::prod(m, v);

  return w;
}


template< typename T >
double determinant (boost::numeric::ublas::matrix< T > const &M) {
  // create a working copy of the input
  boost::numeric::ublas::matrix< T > mLu(M);
  boost::numeric::ublas::permutation_matrix< std::size_t > pivots(M.size1());

  // use uBLAS LU decomposition
  boost::numeric::ublas::lu_factorize(mLu, pivots);

  double det = 1.0;

  // compute the determinant
  for (std::size_t i=0; i < pivots.size(); ++i) {
    if (pivots(i) != i)
      det *= -1.0;
    det *= mLu(i,i);
  }
  return det;
}


#ifdef HAVE_LIBLAPACK
// Helping function-object for gglse.
template< typename T >
struct gglse_
{
  int operator()( integer *m, integer *n, integer *p, T *a, integer *lda,
                  T *b, integer *ldb, T *c__, T *d__, T *x, T *work,
                  integer *lwork, integer *info);
};

// gglse lapack invocation.
template< typename T >
typename gglse_t< T >::Vector gglse_t< T >::operator()
  ( const Matrix &_A, const Matrix &_B,
    const Vector &_c, const Vector &_d ) const
{
  BOOST_STATIC_ASSERT( boost::is_float< T >::value );
  Matrix A( _A ), B( _B );
  Vector c( _c ), d( _d );
  assert( A.size1() == c.size() );
  assert( B.size1() == d.size() );
  assert( A.size2() == B.size2() );
  Vector x( A.size2() );
  integer
    M = A.size1(),
    N = A.size2(),
    P = B.size1(),
    LDA = A.size1(),
    LDB = B.size1(),
    LWORK = -1,
    INFO = 0;
  T DLWORK;
  gglse_< T > gglse_;
  gglse_( &M, &N, &P, A.data().begin(), &LDA, B.data().begin(),
          &LDB, c.data().begin(), d.data().begin(),
          x.data().begin(), &DLWORK, &LWORK, &INFO );
  assert( INFO == 0 );
  LWORK = (integer)DLWORK;
  T WORK[LWORK];
  gglse_( &M, &N, &P, A.data().begin(), &LDA, B.data().begin(),
          &LDB, c.data().begin(), d.data().begin(),
          x.data().begin(), &WORK[0], &LWORK, &INFO );
  assert( INFO == 0 );
  return x;
}

// Helping function objects for inv.
template< typename T >
struct getrf_
{
  int operator()( integer *m, integer *n, T *a, integer *lda, integer *ipiv,
                  integer *info ) const;
};

template< typename T >
struct getri_
{
  int operator()( integer *n, T *a, integer *lda, integer *ipiv, T *work,
                  integer *lwork, integer *info ) const;
};

// inv lapack invocation.
template< typename T >
typename inv_t< T >::Matrix inv_t< T >::operator()( const Matrix &A ) const
{
  BOOST_STATIC_ASSERT( boost::is_float< T >::value );
  assert( A.size1() == A.size2() );
  Matrix AInv( A );
  integer
    N = A.size1(),
    LDA = A.size1(),
    LWORK = -1,
    IPIV[N],
    INFO = 0;
  getrf_< T >()( &N, &N, AInv.data().begin(), &LDA, &IPIV[0], &INFO );
  assert( INFO == 0 );
  T DLWORK;
  getri_< T > getri_;
  getri_( &N, AInv.data().begin(), &LDA, &IPIV[0], &DLWORK, &LWORK,
          &INFO );
  LWORK = (integer)DLWORK;
  T WORK[LWORK];
  getri_( &N, AInv.data().begin(), &LDA, &IPIV[0], &WORK[0], &LWORK,
          &INFO );
  return AInv;
};

// Helping function object for gels
template< typename T >
struct gels_
{
  int operator()( char *trans, integer *m, integer *n, integer *nrhs,
                  T *a, integer *lda, T *b, integer *ldb, T *work,
                  integer *lwork, integer *info ) const;
};

// gels lapack invocation
template< typename T >
typename gels_t<T>::Vector gels_t< T >::operator()
       ( const Matrix &_A, const Vector &_b ) const
{
  BOOST_STATIC_ASSERT( boost::is_float< T >::value );
  Matrix A( _A );
  Vector b( _b );
  assert( A.size1() >= A.size2() );
  assert( A.size1() == b.size() );
  char TRANS = 'N';
  integer
    M = A.size1(),
    N = A.size2(),
    NHRS = 1,
    LDA = A.size1(),
    LDB = b.size(),
    LWORK = -1,
    INFO = 0;
  T DLWORK;
  gels_< T > gels_;
  gels_( &TRANS, &M, &N, &NHRS, A.data().begin(), &LDA,
         b.data().begin(), &LDB, &DLWORK, &LWORK, &INFO );
  assert( INFO == 0 );
  LWORK = (integer)DLWORK;
  T WORK[LWORK];
  gels_( &TRANS, &M, &N, &NHRS, A.data().begin(), &LDA,
         b.data().begin(), &LDB, &WORK[0], &LWORK, &INFO );
  assert( INFO == 0 );
  boost::numeric::ublas::vector< T > x( A.size2() );
  std::copy( b.data().begin(), b.data().begin() + A.size2(),
             x.data().begin() );
  return x;
}

// Helping function object for gesvd
template< typename T >
struct gesvd_
{
  int operator()( char *jobu, char *jobvt, integer *m, integer *n, 
                  T *a, integer *lda, T *s, T *u, integer *ldu, T *vt,
                  integer *ldvt, T *work, integer *lwork, integer *info) const;
};

// gesvd lapack invocation.
template< typename T >
typename gesvd_t< T >::BandedMatrix gesvd_t< T >::operator()
  ( const Matrix &_A, Matrix *_U, Matrix *_Vt ) throw (mimasexception)
{
  BOOST_STATIC_ASSERT( boost::is_float< T >::value );
  char JOBU;
  char JOBVT;
  Matrix A( _A );
  T *U, *Vt;
  if ( _U != NULL ) {
    JOBU = 'A';
    _U->resize( A.size1(), A.size1() );
    U = _U->data().begin();
  } else {
    JOBU = 'N';
    U = NULL;
  }; 
  if ( _Vt != NULL ) {
    JOBVT = 'A';
    _Vt->resize( A.size2(), A.size2() );
    Vt = _Vt->data().begin();
  } else {
    JOBVT = 'N';
    Vt = NULL;
  };
  
  BandedMatrix S( A.size1(), A.size2(), 0, 0 );
  integer
    M = A.size1(),
    N = A.size2(),
    LDA = A.size1(),
    LDU = A.size1(),
    LDVT = A.size2(),
    LWORK = -1,
    INFO = 0;// INFO may be left unmodified by dgesvd/sgesvd.
  gesvd_< T > gesvd_;
  T DLWORK;
  gesvd_( &JOBU, &JOBVT, &M, &N, A.data().begin(), &LDA, S.data().begin(),
          U, &LDU, Vt, &LDVT, &DLWORK, &LWORK, &INFO );
  assert( INFO == 0 );
  LWORK = (integer)DLWORK;
  T WORK[LWORK];
  gesvd_( &JOBU, &JOBVT, &M, &N, A.data().begin(), &LDA, S.data().begin(),
          U, &LDU, Vt, &LDVT, &WORK[0], &LWORK, &INFO );
  assert( INFO >= 0 );
  MMERROR( INFO <= 0, mimasexception, ,
           "gesvd: singular value decomposition didn't converge." );
  return S;
}

// Helping function object for gesdd
template< typename T >
struct gesdd_
{
  int operator()( char *jobz, integer *m, integer *n, T *a, integer *lda,
                  T *s, T *u, integer *ldu, T *vt, integer *ldvt, T *work,
                  integer *lwork, integer *iwork, integer *info );
};

// gesdd lapack invocation.
template< typename T >
typename gesdd_t< T >::BandedMatrix gesdd_t< T >::operator()
  ( const Matrix &_A, Matrix *_U, Matrix *_Vt ) throw (mimasexception)
{
  BOOST_STATIC_ASSERT( boost::is_float< T >::value );
  char JOBZ;
  Matrix A( _A );
  assert( ( _U != NULL ) == ( _Vt != NULL ) );
  T *U, *Vt;
  if ( _U != NULL ) {
    JOBZ = 'A';
    _U->resize( A.size1(), A.size1() );
    _Vt->resize( A.size2(), A.size2() );
    U = _U->data().begin();
    Vt = _Vt->data().begin();
  } else {
    JOBZ = 'N';
    U = NULL;
    Vt = NULL;
  }; 
  
  BandedMatrix S( A.size1(), A.size2(), 0, 0 );
  integer
    M = A.size1(),
    N = A.size2(),
    LDA = A.size1(),
    LDU = A.size1(),
    LDVT = A.size2(),
    LWORK = -1,
    IWORK[ 8 * std::min( M, N ) ],
    INFO = 0;// INFO may be left unmodified by dgesdd/sgesdd.
  gesdd_< T > gesdd_;
#if 0
  T DLWORK;
  gesdd_( &JOBZ, &M, &N, A.data().begin(), &LDA, S.data().begin(),
          U, &LDU, Vt, &LDVT, &DLWORK, &LWORK, &IWORK[0], &INFO );
  assert( INFO == 0 );
  LWORK = (integer)DLWORK;
#else
  // Mandrake's liblapack3-package doesn't contain recent bugfixes of lapack.
  // See http://netlib2.cs.utk.edu/lapack/release_notes.html for bug in sgesdd.
  if ( JOBZ == 'N' )
    LWORK = 3*std::min(M,N) + std::max(std::max(M,N),6*std::min(M,N));
  else {
    assert( JOBZ == 'A' );
    LWORK = 3*std::min(M,N)*std::min(M,N) + std::max(std::max(M,N),4*std::min(M,N)*std::min(M,N)+4*std::min(M,N));
  };    
#endif
  T WORK[LWORK];
  gesdd_( &JOBZ, &M, &N, A.data().begin(), &LDA, S.data().begin(),
          U, &LDU, Vt, &LDVT, &WORK[0], &LWORK, &IWORK[0], &INFO );
  assert( INFO >= 0 );
  MMERROR( INFO <= 0, mimasexception, ,
           "gesdd: singular value decomposition didn't converge." );
  return S;
}

// Helping function for syev
template< typename T >
struct syev_
{
  int operator()( char *jobz, char *uplo, integer *n, T *a,
                  integer *lda, T *w, T *work,
                  integer *lwork, integer *info) const;
};

// syev lapack invocation.
template< typename T >
typename syev_t< T >::EigenValues syev_t< T >::operator()
  ( const SymmetricMatrix &_A, Matrix *_E ) const throw (mimasexception)
{
  BOOST_STATIC_ASSERT( boost::is_float< T >::value );
  char
    JOBZ = _E == NULL ? 'N' : 'V',
    UPLO = 'L';

  boost::shared_ptr< Matrix > A;
  if ( _E == NULL ) {
    A = boost::shared_ptr< Matrix >( new Matrix( _A ) );
    _E = A.get();
  } else
    *_E = _A;

  EigenValues w( _E->size1() );

  integer
    N = _E->size2(),
    LDA = _E->size1(),
    LWORK = -1,
    INFO = 0;
  T DLWORK;
  syev_< T > syev_;
  syev_( &JOBZ, &UPLO, &N, _E->data().begin(), &LDA, w.data().begin(),
         &DLWORK, &LWORK, &INFO );
  assert( INFO == 0 );
  LWORK = (integer)DLWORK;
  T WORK[LWORK];
  syev_( &JOBZ, &UPLO, &N, _E->data().begin(), &LDA, w.data().begin(),
         &WORK[0], &LWORK, &INFO );
  assert( INFO >= 0 );
  MMERROR( INFO <= 0, mimasexception, ,
           "syev: " << INFO << " off-diagonal elements of the "
           "intermediate eigentransform didn't converge." );
  return w;
}

// Helping function object for pptrf
template< typename T >
struct pptrf_
{
  int operator()( char *uplo, integer *n, T *ap, integer *info) const;
};

// pptrf lapack invocation.
template< typename T >
typename pptrf_t< T >::TriangularMatrix pptrf_t< T >::operator()
  ( const SymmetricMatrix &_A ) throw (mimasexception)
{
  BOOST_STATIC_ASSERT( boost::is_float< T >::value );
  SymmetricMatrix A( _A );
  char UPLO = 'U';
  integer
    N = A.size1(),
    INFO = 0;
  pptrf_< T > pptrf_;
  
  pptrf_( &UPLO, &N, A.data().begin(), &INFO);
  MMERROR( INFO == 0, mimasexception, ,
           "pptrf: " << INFO << " unable to compute the "
           "Cholesky decomposition." );
  TriangularMatrix Tg( A );
  return Tg;
}

#endif

};

#endif

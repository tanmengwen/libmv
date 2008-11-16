#include "linalg.h"

namespace mimas {

#ifdef HAVE_LIBLAPACK

template<>
int getrf_< double >::operator()( integer *m, integer *n, double *a,
                                  integer *lda, integer *ipiv, integer *info )
  const
{
  return dgetrf_( m, n, a, lda, ipiv, info );
}

template<>
int getrf_< float >::operator()( integer *m, integer *n, float *a,
                                 integer *lda, integer *ipiv, integer *info )
  const
{
  return sgetrf_( m, n, a, lda, ipiv, info );
}

template<>
int getri_< double >::operator()( integer *n, double *a, integer *lda,
                                  integer *ipiv, double *work,
                                  integer *lwork, integer *info ) const
{
  return dgetri_( n, a, lda, ipiv, work, lwork, info );
}

template<>
int getri_< float >::operator()( integer *n, float *a, integer *lda,
                                 integer *ipiv, float *work,
                                 integer *lwork, integer *info ) const
{
  return sgetri_( n, a, lda, ipiv, work, lwork, info );
}

template<>
int gglse_< double >::operator()( integer *m, integer *n, integer *p,
                                  double *a, integer *lda, double *b,
                                  integer *ldb, double *c__, double *d__,
                                  double *x, double *work, integer *lwork,
                                  integer *info )
{
  return dgglse_( m, n, p, a, lda, b, ldb, c__, d__, x, work, lwork, info );
}

template<>
int gglse_< float >::operator()( integer *m, integer *n, integer *p,
                                 float *a, integer *lda, float *b,
                                 integer *ldb, float *c__, float *d__,
                                 float *x, float *work, integer *lwork,
                                 integer *info )
{
  return sgglse_( m, n, p, a, lda, b, ldb, c__, d__, x, work, lwork, info );
}

template<>
int gels_< double >::operator()( char *trans, integer *m, integer *n,
                                 integer *nrhs, double *a, integer *lda,
                                 double *b, integer *ldb, double *work,
                                 integer *lwork, integer *info ) const
{
  return dgels_( trans, m, n, nrhs, a, lda, b, ldb, work, lwork, info );
}

template<>
int gels_< float >::operator()( char *trans, integer *m, integer *n,
                                integer *nrhs, float *a, integer *lda,
                                float *b, integer *ldb, float *work,
                                integer *lwork, integer *info ) const
{
  return sgels_( trans, m, n, nrhs, a, lda, b, ldb, work, lwork, info );
}

template<>
int gesvd_< double >::operator()( char *jobu, char *jobvt, integer *m,
                                  integer *n, double *a, integer *lda,
                                  double *s, double *u, integer *ldu,
                                  double *vt, integer *ldvt, double *work,
                                  integer *lwork, integer *info) const
{
  return dgesvd_( jobu, jobvt, m, n, a, lda, s, u, ldu, vt, ldvt, work, lwork,
                  info );
}

template<>
int gesvd_< float >::operator()( char *jobu, char *jobvt, integer *m,
                                 integer *n, float *a, integer *lda,
                                 float *s, float *u, integer *ldu,
                                 float *vt, integer *ldvt, float *work,
                                 integer *lwork, integer *info) const
{
  return sgesvd_( jobu, jobvt, m, n, a, lda, s, u, ldu, vt, ldvt, work, lwork,
                  info );
}

template<>
int gesdd_< double >::operator()( char *jobz, integer *m, integer *n,
                                  double *a, integer *lda, double *s,
                                  double *u, integer *ldu, double *vt,
                                  integer *ldvt, double *work, integer *lwork,
                                  integer *iwork, integer *info )
{
  return dgesdd_( jobz, m, n, a, lda, s, u, ldu, vt, ldvt, work, lwork,
                  iwork, info );
}

template<>
int gesdd_< float >::operator()( char *jobz, integer *m, integer *n,
                                 float *a, integer *lda, float *s,
                                 float *u, integer *ldu, float *vt,
                                 integer *ldvt, float *work, integer *lwork,
                                 integer *iwork, integer *info )
{
  return sgesdd_( jobz, m, n, a, lda, s, u, ldu, vt, ldvt, work, lwork,
                  iwork, info );
}

template<>
int syev_< double >::operator()( char *jobz, char *uplo, integer *n,
                                 double *a, integer *lda, double *w,
                                 double *work, integer *lwork,
                                 integer *info)
  const
{
  return dsyev_( jobz, uplo, n, a, lda, w, work, lwork, info );
}

template<>
int syev_< float >::operator()( char *jobz, char *uplo, integer *n,
                                float *a, integer *lda, float *w,
                                float *work, integer *lwork,
                                integer *info )
  const
{
  return ssyev_( jobz, uplo, n, a, lda, w, work, lwork, info );
}

template<>
int pptrf_< double >::operator()( char *uplo, integer *n, double *ap,
                                  integer *info) const
{
  return dpptrf_( uplo, n, ap, info );
}

template<>
int pptrf_< float >::operator()( char *uplo, integer *n, float *ap,
                                  integer *info) const
{
  return spptrf_( uplo, n, ap, info );
}

#endif

};

/////////////////////////////////////////////////////////////////////////////////
// 
//  Non-linear, robust homography estimation
//  Copyright (C) 2003-06  Manolis Lourakis (lourakis@ics.forth.gr)
//  Institute of Computer Science, Foundation for Research & Technology - Hellas
//  Heraklion, Crete, Greece.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
/////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>
#include <float.h>

#include "util.h"

/* eigenvalues & eigenvectors */
extern int dspev_(char *jobz, char *uplo, int *n, double *ap, 
                  double *w, double *z, int *ldz, double *work, int *info);

/* SVD */
extern int dgesvd_(char *jobu, char *jobvt, int *m, int *n,
                   double *a, int *lda, double *s, double *u, int *ldu,
                   double *vt, int *ldvt, double *work, int *lwork, int *info);

/* lapack 3.0 routine, faster than dgesvd() */
extern int dgesdd_(char *jobz, int *m, int *n, double *a, int *lda,
                   double *s, double *u, int *ldu, double *vt, int *ldvt,
                   double *work, int *lwork, int *iwork, int *info);


/* Solve min |Ax| subject to |x|=1
 * The solution is the eigenvector of A^T A corresponding
 * to the smallest eigenvalue.
 *
 * A is mxn, x is nx1
 *
 * The function returns 0 in case of error, 1 if successfull
 *
 * This function is often called repetitively to solve problems of identical
 * dimensions. To avoid repetitive malloc's and free's, allocated memory is
 * retained between calls and free'd-malloc'ed when not of the appropriate size.
 * A call with NULL as the first argument forces this memory to be released.
 */
int min_Ax_normEIG(double *A, int m, int n, double *x)
{
static double *buf=NULL;
static int buf_sz=0;

register int i, j, k;
double *triang, *eigvals, *eigvecs, *work, thresh;
register double sum;
int info, tot_sz, triang_sz, eigvals_sz, eigvecs_sz, work_sz;

  if(!A){
    if(buf) free(buf);
    buf=NULL;
    buf_sz=0;
    return 1;
  }

  /* calculate required memory size */
  triang_sz=(n*(n+1)/2);
  eigvals_sz=n;
  eigvecs_sz=n*n;
  work_sz=3*n;
  tot_sz=triang_sz+eigvals_sz+eigvecs_sz+work_sz;

  if(tot_sz>buf_sz){ /* insufficient memory, allocate a "big" memory chunk at once */
    if(buf) free(buf); /* free previously allocated memory */

    buf_sz=tot_sz;
    buf=(double *)malloc(buf_sz*sizeof(double));
    if(!buf){
      fprintf(stderr, "Memory allocation request in min_Ax_normEIG() failed!\n");
      exit(1);
    }
  }

  triang=buf;
  eigvals=triang+triang_sz;
  eigvecs=eigvals+eigvals_sz;
  work=eigvecs+eigvecs_sz;

  /* triang = A^t * A */
  for(j=0; j<n; j++)
    for(i=0; i<=j; i++){
      for(k=0, sum=0.0; k<m; k++)
        sum+=A[k*n+i]*A[k*n+j];
      triang[i + ((j*(j+1))>> 1)]=sum; //i + j*(j+1)/2
    }

  dspev_("V", "U", &n, triang, eigvals, eigvecs, &n, work, &info);

  if(info<0){
    fprintf(stderr, "LAPACK error: illegal value for argument %d of dspev in min_Ax_normEIG()\n", -info);
    exit(1);
  }
  else if(info>0){
    fprintf(stderr, "LAPACK error: dspev failed to converge in min_Ax_normEIG();\n%d %s", info,
        "off-diagonal elements of an intermediate tridiagonal form did not converge to zero\n");
    return 0;
  }

  for(i=n-1, j=0, thresh=eigvals[n-1]*DBL_EPSILON; i>0; --i, ++j)
    if(eigvals[i]<=thresh) break; /* remaining eigenvalues are smaller than this */


  if(j!=n-1){ /* matrix is not of rank n-1! */
    //fprintf(stderr, "Unacceptable rank %d in min_Ax_normEIG\n", rank);
    return 0;
  }

  /* min eigenvalue is the first (they are returned in ascending order), return the first eigenvector */
  for(i=0; i<n; i++)
    x[i]=eigvecs[i];

  return 1;
}


/* Solve min |Ax| subject to |x|=1
 * The solution is the right singular vector (Vt) corresponding to A's
 * minimum singular value: A=U*D*V^T ==> A^T*A=V*D^2*V^T
 *
 * A is mxn, x is nx1
 *
 * The function returns 0 in case of error, 1 if successfull
 *
 * This function is often called repetitively to solve problems of identical
 * dimensions. To avoid repetitive malloc's and free's, allocated memory is
 * retained between calls and free'd-malloc'ed when not of the appropriate size.
 * A call with NULL as the first argument forces this memory to be released.
 */
int min_Ax_normSVD(double *A, int m, int n, double *x)
{
static double *buf=NULL;
static int buf_sz=0;

register int i, j;
double *a, *u, *s, *vt, *work, thresh;
int info, worksz, *iwork, iworksz;
int a_sz, u_sz, s_sz, vt_sz, tot_sz;

  if(!A){
    if(buf) free(buf);
    buf=NULL;
    buf_sz=0;
    return 1;
  }

  /* calculate required memory size. Note that if dgesvd is used, the memory for u is not actually needed... */
  worksz=-1; // workspace query. Keep in mind that dgesdd requires more memory than dgesvd
  /* note that optimal work size is returned in thresh */
  //dgesdd_("A", (int *)&m, (int *)&n, NULL, (int *)&m, NULL, NULL, (int *)&m, NULL, (int *)&n, (double *)&thresh, (int *)&worksz, NULL, &info);
  dgesvd_("N", "A", (int *)&m, (int *)&n, NULL, (int *)&m, NULL, NULL, (int *)&m, NULL, (int *)&n, (double *)&thresh, (int *)&worksz, &info);
  worksz=(int)thresh;
  iworksz=8*n;
  a_sz=m*n;
  u_sz=m*m; s_sz=n; vt_sz=n*n;

  tot_sz=iworksz*sizeof(int) + (a_sz + u_sz + s_sz + vt_sz + worksz)*sizeof(double);

  if(tot_sz>buf_sz){ /* insufficient memory, allocate a "big" memory chunk at once */
    if(buf) free(buf); /* free previously allocated memory */

    buf_sz=tot_sz;
    buf=(double *)malloc(buf_sz);
    if(!buf){
      fprintf(stderr, "Memory allocation request in min_Ax_normSVD() failed!\n");
      exit(1);
    }
  }

  iwork=(int *)buf;
  a=(double *)(iwork+iworksz);
  /* store A (column major!) into a */
  for(i=0; i<m; i++)
    for(j=0; j<n; j++)
      a[i+j*m]=A[i*n+j];

  u=((double *)(iwork+iworksz)) + a_sz;
  s=u+u_sz;
  vt=s+s_sz;
  work=vt+vt_sz;

  /* SVD decomposition of A */
  //dgesdd_("A", (int *)&m, (int *)&n, a, (int *)&m, s, u, (int *)&m, vt, (int *)&n, work, (int *)&worksz, iwork, &info);
  dgesvd_("N", "A", (int *)&m, (int *)&n, a, (int *)&m, s, NULL, (int *)&m, vt, (int *)&n, work, (int *)&worksz, &info);

  /* error treatment */
  if(info!=0){
    if(info<0){
      fprintf(stderr, "LAPACK error: illegal value for argument %d of dgesdd in min_Ax_normSVD()\n", -info);
      exit(1);
    }
    else{
      fprintf(stderr, "LAPACK error: dgesdd (dbdsdc)/dgesvd (dbdsqr) failed to converge in min_Ax_normSVD() [info=%d]\n", info);
      return 0;
    }
  }

  /* determine A's rank */
  for(i=0, thresh=DBL_EPSILON*s[0]; i<n; ++i)
    if(s[i]<=thresh) break; /* remaining singular values are smaller than this */

  if(i<n-1) 
    return 0; /* A should have rank n-1 */

  /* s[n-1] is the smallest singular value */
  vt+=n-1;
  for(j=0; j<n; ++j)
    x[j]=vt[j*n]; //vt[n-1+j*n];

  return 1;
}

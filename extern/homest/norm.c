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
#include <math.h>
#include <float.h>

#include "util.h"

#ifndef M_SQRT2
#define M_SQRT2  1.41421356237309504880  /* sqrt(2) */
#endif

#define HYPOT(a, b)     sqrt((a)*(a) + (b)*(b))   /* hypot(a, b) */

#define MAT3x3_TRANSPOSE_IN_PLACE(M){ \
      double tmp__;                   \
                                      \
      tmp__=M[1];                     \
      M[1]=M[3];                      \
      M[3]=tmp__;                     \
                                      \
      tmp__=M[2];                     \
      M[2]=M[6];                      \
      M[6]=tmp__;                     \
                                      \
      tmp__=M[5];                     \
      M[5]=M[7];                      \
      M[7]=tmp__;                     \
}


/*
 * This function computes the inverse of a 3x3 matrix A into A1
 * using determinants
 *
 * The function returns 0 in case of error (e.g. A is singular),
 * 1 if successfull
 */
#define MAT3x3_COEF(M, i, j) ((M) + (i)*3+(j))
int mat3x3Inverse(double *A, double *A1)
{
double det;

	det = *MAT3x3_COEF(A, 0, 0) * (*MAT3x3_COEF(A, 1, 1) * *MAT3x3_COEF(A, 2, 2) - *MAT3x3_COEF(A, 2, 1) * *MAT3x3_COEF(A, 1, 2)) -
	      *MAT3x3_COEF(A, 0, 1) * (*MAT3x3_COEF(A, 1, 0) * *MAT3x3_COEF(A, 2, 2) - *MAT3x3_COEF(A, 1, 2) * *MAT3x3_COEF(A, 2, 0)) +
	      *MAT3x3_COEF(A, 0, 2) * (*MAT3x3_COEF(A, 1, 0) * *MAT3x3_COEF(A, 2, 1) - *MAT3x3_COEF(A, 2, 0) * *MAT3x3_COEF(A, 1, 1));

	if (-DBL_MIN<=det && det<=DBL_MIN){
		fprintf(stderr, "Zero determinant (%g) in mat3x3Inverse()\n", det);
    return 0;
	}

  det=1.0/det;
	*MAT3x3_COEF(A1, 0, 0)= (*MAT3x3_COEF(A, 1, 1) * *MAT3x3_COEF(A, 2, 2) - *MAT3x3_COEF(A, 2, 1) * *MAT3x3_COEF(A, 1, 2)) * det;
	*MAT3x3_COEF(A1, 0, 1)= (*MAT3x3_COEF(A, 2, 1) * *MAT3x3_COEF(A, 0, 2) - *MAT3x3_COEF(A, 0, 1) * *MAT3x3_COEF(A, 2, 2)) * det;
	*MAT3x3_COEF(A1, 0, 2)= (*MAT3x3_COEF(A, 0, 1) * *MAT3x3_COEF(A, 1, 2) - *MAT3x3_COEF(A, 1, 1) * *MAT3x3_COEF(A, 0, 2)) * det;
	*MAT3x3_COEF(A1, 1, 0)= (*MAT3x3_COEF(A, 1, 2) * *MAT3x3_COEF(A, 2, 0) - *MAT3x3_COEF(A, 1, 0) * *MAT3x3_COEF(A, 2, 2)) * det;
	*MAT3x3_COEF(A1, 1, 1)= (*MAT3x3_COEF(A, 0, 0) * *MAT3x3_COEF(A, 2, 2) - *MAT3x3_COEF(A, 0, 2) * *MAT3x3_COEF(A, 2, 0)) * det;
	*MAT3x3_COEF(A1, 1, 2)= (*MAT3x3_COEF(A, 0, 2) * *MAT3x3_COEF(A, 1, 0) - *MAT3x3_COEF(A, 0, 0) * *MAT3x3_COEF(A, 1, 2)) * det;
	*MAT3x3_COEF(A1, 2, 0)= (*MAT3x3_COEF(A, 1, 0) * *MAT3x3_COEF(A, 2, 1) - *MAT3x3_COEF(A, 1, 1) * *MAT3x3_COEF(A, 2, 0)) * det;
	*MAT3x3_COEF(A1, 2, 1)= (*MAT3x3_COEF(A, 0, 1) * *MAT3x3_COEF(A, 2, 0) - *MAT3x3_COEF(A, 0, 0) * *MAT3x3_COEF(A, 2, 1)) * det;
	*MAT3x3_COEF(A1, 2, 2)= (*MAT3x3_COEF(A, 0, 0) * *MAT3x3_COEF(A, 1, 1) - *MAT3x3_COEF(A, 0, 1) * *MAT3x3_COEF(A, 1, 0)) * det;

  return 1;
}
#undef MAT3x3_COEF

/* C=A*B */
static void mat3x3Mult(double A[9], double B[9], double C[9])
{
register int i, j;
register double sum;

  for(i=0; i<3; ++i)
    for(j=0; j<3; ++j){
      sum= A[i*3]*B[j];
      sum+=A[i*3+1]*B[3+j];
      sum+=A[i*3+2]*B[6+j];
      C[i*3+j]=sum;
    }
}


/* Compute a normalizing transformation T mapping 'pts' to 'npts' as suggested by Hartley */
void normalizePts(double (*pts)[2], double (*npts)[2], int numpts, double T[9])
{
register int i;
double centx, centy;
double dist, scale;

	centx=centy=0.0;
	for(i=0; i<numpts; i++){
		centx+=pts[i][0];
		centy+=pts[i][1];
	}

	centx/=(double)(numpts);
	centy/=(double)(numpts);

	for(i=0; i<numpts; i++){
    npts[i][0]=pts[i][0]-centx;
    npts[i][1]=pts[i][1]-centy;
	}

	dist=0.0;
	for(i=0; i<numpts; i++){
		dist+=HYPOT(npts[i][0], npts[i][1]);
	}
		
	dist/=(double)(numpts);
  scale=M_SQRT2/dist;
	
	for(i=0; i<numpts; i++){
    npts[i][0]*=scale;
    npts[i][1]*=scale;
	}

	T[0]=scale; T[1]=0.0;   T[2]=-centx*scale;
	T[3]=0.0;   T[4]=scale; T[5]=-centy*scale;
	T[6]=0.0;   T[7]=0.0;   T[8]=1.0;
}

/* xpt = T*pt */
void xformPt(double pt[2], double T[9], double xpt[2])
{
double tmp;

  xpt[0]=T[0]*pt[0] + T[1]*pt[1] + T[2];
  xpt[1]=T[3]*pt[0] + T[4]*pt[1] + T[5];
  tmp   =T[6]*pt[0] + T[7]*pt[1] + T[8];

  xpt[0]/=tmp;
  xpt[1]/=tmp;
}

#if 0
/* nfm=T2^-t * FM * T1^-1  */
void normalizeFM(double fm[9], double T1[9], double T2[9], double nfm[9])
{
double T1_1[9], T2_1[9], tmp[9];

  mat3x3Inverse(T1, T1_1);
  mat3x3Inverse(T2, T2_1);

	MAT3x3_TRANSPOSE_IN_PLACE(T2_1);

  mat3x3Mult(T2_1, fm, tmp);  /* tmp <-- T2^-t * FM       */
  mat3x3Mult(tmp, T1_1, nfm); /* nfm<-- T2^-t * FM * T1^-1 */
}

/* Fm = T2^t * nFm * T1 */
void denormalizeFM(double nFm[9], double T1[9], double T2[9], double Fm[9])
{
double tmp[9];

	MAT3x3_TRANSPOSE_IN_PLACE(T2);
	mat3x3Mult(T2, nFm, tmp); /* tmp <-- T2^t * nFm     */
	mat3x3Mult(tmp, T1, Fm);  /* Fm <-- T2^t * nFm * T1 */

	MAT3x3_TRANSPOSE_IN_PLACE(T2);    /* undo transpose */
}
#endif /* 0 */

/* nH = T2 * H * T1^-1 */
void normalizeH(double H[9], double T1[9], double T2[9], double nH[9])
{
double T1_1[9], tmp[9];

	mat3x3Inverse(T1, T1_1);
  mat3x3Mult(T2, H, tmp);    /* tmp <-- T2 * H   */
  mat3x3Mult(tmp, T1_1, nH); /* nH <-- T2 * H * T1^-1 */
}

/* H <-- T2^-1 * nH * T1 */
void denormalizeH(double nH[9], double T1[9], double T2[9], double H[9])
{
double T2_1[9], tmp[9];

	mat3x3Inverse(T2, T2_1);
  mat3x3Mult(T2_1, nH, tmp); /* tmp <-- T2^-1 * nH */
  mat3x3Mult(tmp, T1, H);    /* H <-- T2^-1 * nH * T1 */
}


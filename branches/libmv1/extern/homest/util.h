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

#ifndef _UTIL_H
#define _UTIL_H

#define MIN_HMATCHED_PTS    4

/* min_Ax_norm.c */
extern int min_Ax_normEIG(double *A, int m, int n, double *x);
extern int min_Ax_normSVD(double *A, int m, int n, double *x);

/* norm.c */
extern void normalizePts(double (*pts)[2], double (*npts)[2], int numpts, double T[9]);
extern void xformPt(double pt[2], double T[9], double xpt[2]);
//extern void normalizeFM(double fm[9], double T1[9], double T2[9], double nfm[9]);
//extern void denormalizeFM(double nFm[9], double T1[9], double T2[9], double Fm[9]);
extern void normalizeH(double H[9], double T1[9], double T2[9], double nH[9]);
extern void denormalizeH(double nH[9], double T1[9], double T2[9], double H[9]);
extern int mat3x3Inverse(double *A, double *A1);

/* buckets.c */
extern int genRandomSetsNoBuckets(int sizeSet, int nbData, int nbSets, int **sets);
extern int genRandomSetsWithBuckets(double (*pts)[2], int sizeSet, int nbData, int nbSets, int **sets);

#endif /* _UTIL_H */

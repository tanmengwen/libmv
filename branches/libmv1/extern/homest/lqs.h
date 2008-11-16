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

#ifndef _LQS_H
#define _LQS_H

extern int lqs_numtries(int p, double i, double P);
extern int **lqs_allocsets(int sizeSet, int nbSets);
extern void lqs_freesets(int **sets);
extern void lqs_initnoise();
extern void lqs_setrepeatablerandom(int flag);
extern double lqs_uniformnoise();

extern int lqsfit(int nbData, int sizeSet, int **sets, int nbSets,
                  void (*residual)(double *x, int nb, double *resid),
                  int (*estimator)(double *x, int nb, int *indexes),
                  int isResidualSqr, int verbose, int maxNbSol, double gate,
                  double prematureResidual, int dim, double percentageOfGoodData, double *estimate,
                  int *bestSet, int *outliersMap, int *nbOutliers, double *outlierThresh);

extern double kth_smallest(double *a, int n, int k);

#endif /* _LQS_H */

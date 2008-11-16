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

#include "util.h"
#include "homest.h"
#include "maplefuncs.h"

#include "lqs.h"
#include <lm.h>

#define USE_BUCKETS

#define HOWTO_CALC_RESIDUALS homogResidualsGeom   // geometric distances
//#define HOWTO_CALC_RESIDUALS homogResidualsAlg    // algebraic distances

/* global variables used by various homography estimation routines */
static struct {
  double **eqs;
  double (*pts0)[2], (*pts1)[2];
  int *inliersidx, numInliers;

  /* following elements are used for avoiding multiple mallocs in est2DPtHomog() */
  double *M;
  int Mrows;
} globs;

/* estimate "point" homography H s.t. p'=H*p, with p, p' specified by ptsidx */
static int est2DPtHomog(double *h, int npts, int *ptsidx)
{
register int i, j;
double *mat, *matrow, *coefs;

  if(globs.Mrows!=2*npts){ /* free existing matrix and allocate a new one */
    if(globs.M) /* is there anything to free? */
      free(globs.M);

    if((mat=(double *)malloc(2*npts*NUM_HPARAMS*sizeof(double)))==NULL){
      fprintf(stderr, "Memory allocation request failed in est2DPtHomog()\n");
      exit(1);
    }
    globs.M=mat;
    globs.Mrows=2*npts;
  }

  mat=globs.M;

  /* fill in constraints matrix row by row */
  for(i=0; i<npts; ++i){
    matrow=mat+2*i*NUM_HPARAMS;
    coefs=globs.eqs[ptsidx[i]];
    for(j=0; j<2*NUM_HPARAMS; ++j) // 2 rows for each point pair
      matrow[j]=coefs[j];
  }

  /* solve min |mat*h|, |h|=1 */
  if(!min_Ax_normSVD(mat, 2*npts, NUM_HPARAMS, h))
    return 0;

  return 1;
}

/* compute the (squared) algebraic residuals corresponding to homog */
static void homogResidualsAlg(double *homog, int numres, double *resid)
{
register int i, j;
double *eq1, *eq2, sum1, sum2;

  for(i=0; i<numres; ++i){
    eq1=globs.eqs[i];
    eq2=eq1+NUM_HPARAMS;
    sum1=sum2=0.0;
    for(j=0; j<NUM_HPARAMS; ++j){
        sum1+=eq1[j]*homog[j];
        sum2+=eq2[j]*homog[j];
    }
    resid[i]=sum1*sum1 + sum2*sum2;
  }
}

/* compute the geometric residuals corresponding to homog */
static void homogResidualsGeom(double *homog, int numres, double *resid)
{
register int i;
double *pt0, *pt1;

  for(i=0; i<numres; ++i){
    pt0=globs.pts0[i];
    pt1=globs.pts1[i];

    calc2DHomogNonLinErr(pt0, pt1, homog, resid+i);
  }
}

/* symmetric transfer error and jacobian */
static void homNLerr(double *h, double *x, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
double (*pts1)[2]=globs.pts1;

  for(i=0; i<ninl; ++i){
    calc2DHomogNonLinErr(pts0[inlidx[i]], pts1[inlidx[i]], h, x+i);
  }
}

static void jachomNLerr(double *h, double *jac, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
double (*pts1)[2]=globs.pts1;
register double *jacrow;

  for(i=0, jacrow=jac; i<ninl; ++i, jacrow+=NUM_HPARAMS){
    calc2DHomogNonLinErrGrads(pts0[inlidx[i]], pts1[inlidx[i]], h, jacrow);
  }
}

/* Sampson error and jacobian */
static void homSampsonerr(double *h, double *x, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
double (*pts1)[2]=globs.pts1;

  for(i=0; i<ninl; ++i){
    calc2DHomogSampsonErr(pts0[inlidx[i]], pts1[inlidx[i]], h, x+i);
  }
}

static void jachomSampsonerr(double *h, double *jac, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
double (*pts1)[2]=globs.pts1;
register double *jacrow;

  for(i=0, jacrow=jac; i<ninl; ++i, jacrow+=NUM_HPARAMS){
    calc2DHomogSampsonErrGrads(pts0[inlidx[i]], pts1[inlidx[i]], h, jacrow);
  }
}

/* nonlinear refinement of a homography; based on minimizing symmetric transfer error */
static void refine2DPtHomog(double *h, int howto, int verbose)
{
register int i;
double opts[LM_OPTS_SZ], info[LM_INFO_SZ], *x;
int m=NUM_HPARAMS, n=globs.numInliers; // # unknowns & # constraints
void (*err)(double *p, double *hx, int m, int n, void *adata);
void (*jacerr)(double *p, double *j, int m, int n, void *adata);
int ret;

  if((x=(double *)malloc(n*sizeof(double)))==NULL){ // one equation per point pair
     fprintf(stderr, "Memory allocation request failed in refine2DPtHomog()\n");
     exit(1);
  }
  for(i=0; i<n; ++i) x[i]=0.0;

  opts[0]=LM_INIT_MU; opts[1]=1E-11; opts[2]=1E-11; opts[3]=1E-15;
  opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference jacobian version is used 

  switch(howto){
    case HOMEST_SYM_XFER_ERROR:
      err=homNLerr;
      jacerr=jachomNLerr;
    break;

    case HOMEST_SAMPSON_ERROR:
      err=homSampsonerr;
      jacerr=jachomSampsonerr;
    break;

    default:
      fprintf(stderr, "unknown non-linear homography refinement choice made in refine2DPtHomog(), exiting\n");
      exit(1);
  }

  ret=dlevmar_der(err, jacerr, h, x, m, n, 1000, opts, info, NULL, NULL, NULL); // with analytic jacobian
  //ret=dlevmar_dif(err, h, x, m, n, 1000, opts, info, NULL, NULL, NULL); // no jacobian
  if(verbose){
    printf("\nLM returned %d in %g iter, reason %g\nSolution: ", ret, info[5], info[6]);
    for(i=0; i<m; ++i)
      printf("%.7g ", h[i]);
    printf("\n\nMinimization info:\n");
    for(i=0; i<LM_INFO_SZ; ++i)
      printf("%g ", info[i]);
    printf("\n");
  }

  free(x);
}

/* robust, non-linear homography estimation from "nmatches" matched point features, possibly
 * including outliers. "inpts0", "inpts1" contain the matched image point coordinates, "inlPcent"
 * is the expected percentage of inliers (>=0.5), "H01" contains the estimated homography upon
 * return, "normalize" is a flag specifing whether input coordinates should be normalized according
 * to Hartley's suggestion, "NLrefine" specifies which cost function should be employed for the
 * non-linear refinement step (see homest.h for appropriate values), "nbOutliers" contains the
 * number of detected outliers upon return, "verbose" specifies the verbosity level
 */
HOMEST_API_MOD void HOMEST_CALL_CONV
homest(double (*inpts0)[2], double (*inpts1)[2], int nmatches, double inlPcent, double H01[NUM_HPARAMS],
            int normalize, int NLrefine, int *nbOutliers, int verbose)
{
register int i, j;
int isSqr=1, maxNbSol=1;
double gate=2.0, premResid=-1.0, sampleProb=0.99, outlierThresh;
int *outliersMap, ret, **sets=NULL, nbSets=0;
double (*pts0)[2], (*pts1)[2], L0[9], L1[9];

  if(normalize){
    pts0=(double (*)[2])malloc(nmatches*sizeof(double[2]));
    pts1=(double (*)[2])malloc(nmatches*sizeof(double[2]));
    if(!*pts0 || !*pts1){
      fprintf(stderr, "Momory allocation request failed in homest()\n");
      exit(1);
    }
    normalizePts(inpts0, pts0, nmatches, L0);
    normalizePts(inpts1, pts1, nmatches, L1);
  }
  else{
    pts0=inpts0;
    pts1=inpts1;
  }

  globs.eqs=(double **)malloc(nmatches*sizeof(double *));
  if(!globs.eqs){
    fprintf(stderr, "Error: not enough memory for 'globs.eqs' in homest()\n");
    exit(1);
  }

  /* one "big" malloc instead of several "small" ones */
  globs.eqs[0]=(double *)malloc(2*nmatches*NUM_HPARAMS*sizeof(double)); /* two equations per point */
  if(!globs.eqs[0]){
    fprintf(stderr, "Error: not enough memory for 'globs.eqs[0]' in homest()\n");
    exit(1);
  }
	for(i=1; i<nmatches; i++)
    globs.eqs[i]=globs.eqs[i-1] + 2*NUM_HPARAMS;

	for(i=0; i<nmatches; ++i)
    calc2DHomogLinCoeffs2(pts0[i], pts1[i], globs.eqs[i]);

  nbSets=lqs_numtries(MIN_HMATCHED_PTS, inlPcent, sampleProb);
  sets=lqs_allocsets(MIN_HMATCHED_PTS, nbSets);

#ifdef USE_BUCKETS
  nbSets=genRandomSetsWithBuckets(pts1, MIN_HMATCHED_PTS, nmatches, nbSets, sets);
#else
  nbSets=genRandomSetsNoBuckets(MIN_HMATCHED_PTS, nmatches, nbSets, sets);
#endif /* USE_BUCKETS */

  globs.pts0=pts0; globs.pts1=pts1;
  globs.M=NULL; globs.Mrows=0;

  if(!(outliersMap=(int *)malloc(nmatches*sizeof(int)))){
    fprintf(stderr, "Error: not enough memory for 'outliersMap' in homest()\n");
    exit(1);
  }
  verbose=verbose>1;
  ret=lqsfit(nmatches, MIN_HMATCHED_PTS, sets, nbSets, HOWTO_CALC_RESIDUALS, est2DPtHomog,
            isSqr, verbose, maxNbSol, gate, premResid, NUM_HPARAMS, inlPcent, H01,
            NULL, outliersMap, nbOutliers, &outlierThresh);

  if(verbose){
    fprintf(stderr, "Outlier threshold: %g\n", outlierThresh);
    fprintf(stderr, "homest(): LQS fit returned %d, %d outliers [out of %d]\n", ret, *nbOutliers, nmatches);
  }

  if(sets) lqs_freesets(sets);

  globs.inliersidx=(int *)malloc((nmatches - *nbOutliers)*sizeof(int));
  if(!globs.inliersidx){
    fprintf(stderr, "Error: not enough memory for 'globs.inliersidx' in homest()\n");
    exit(1);
  }

  for(i=j=0; i<nmatches; ++i)
   if(!outliersMap[i]) globs.inliersidx[j++]=i;

  /* LS estimation on inliers */
  globs.numInliers=nmatches - *nbOutliers;
  est2DPtHomog(H01, globs.numInliers, globs.inliersidx);

  /* free working memory */
  free(globs.M);
  globs.M=NULL; globs.Mrows=0;

  if(normalize){
    free(pts0);
    free(pts1);
    denormalizeH(H01, L0, L1, H01);
  }

  /* non linear refinement */
  if(globs.numInliers>=NUM_HPARAMS && NLrefine!=HOMEST_NO_NLN_REFINE){
    /* use the unnormalized points for the refinement */
    globs.pts0=inpts0; globs.pts1=inpts1;
    refine2DPtHomog(H01, NLrefine, verbose);
  }

  /* just in case ... */
  globs.pts0=globs.pts1=NULL;
  globs.numInliers=0;

  free(globs.inliersidx);
  free(globs.eqs[0]);
	free(globs.eqs);
  free(outliersMap);

  globs.eqs=NULL;
  globs.inliersidx=NULL;
}

/* Compute the Root Mean Squared (RMS) and Root Median Squared (RMedS) symmetric average distance
 * error pertaining to a homography H that has been estimated from pairs of corresponding points.
 * Note that the RMS measure is sensitive to mismatched points while the RMedS is not
 */

#define MEDIAN(a, n) kth_smallest(a, n, (((n)&1)? ((n)/2) : (((n)/2)-1)))

HOMEST_API_MOD void HOMEST_CALL_CONV
homest_RMS_RMedS(double (*inpts0)[2], double (*inpts1)[2], int nmatches, double H[NUM_HPARAMS],
                 double *rms, double *rmeds)
{
register int i;
double H1[NUM_HPARAMS], *errors, sum, q1[2], q2[2], s, dist1, dist2;

  if((errors=(double *)malloc(nmatches*sizeof(double)))==NULL){
     fprintf(stderr, "Memory allocation request failed in homest_RMS_RMedS()\n");
     exit(1);
  }
  mat3x3Inverse(H, H1);

  for(i=0, sum=0.0; i<nmatches; i++){
      /* transform inpts1[i] according to H^-1 */
      q1[0]=H1[0]*inpts1[i][0]+H1[1]*inpts1[i][1]+H1[2];
      q1[1]=H1[3]*inpts1[i][0]+H1[4]*inpts1[i][1]+H1[5];
      s    =H1[6]*inpts1[i][0]+H1[7]*inpts1[i][1]+H1[8];
      q1[0]/=s; q1[1]/=s;

      /* transform inpts0[i] according to H */
      q2[0]=H[0]*inpts0[i][0]+H[1]*inpts0[i][1]+H[2];
      q2[1]=H[3]*inpts0[i][0]+H[4]*inpts0[i][1]+H[5];
      s    =H[6]*inpts0[i][0]+H[7]*inpts0[i][1]+H[8];
      q2[0]/=s; q2[1]/=s;

      dist1=(q1[0]-inpts0[i][0])*(q1[0]-inpts0[i][0]) + (q1[1]-inpts0[i][1])*(q1[1]-inpts0[i][1]);
      dist2=(q2[0]-inpts1[i][0])*(q2[0]-inpts1[i][0]) + (q2[1]-inpts1[i][1])*(q2[1]-inpts1[i][1]);
      sum+=errors[i]=(dist1+dist2)/2.0;
  }

  *rms=sqrt(sum/(double)(nmatches));
  *rmeds=sqrt(MEDIAN(errors, nmatches));

  free(errors);
}

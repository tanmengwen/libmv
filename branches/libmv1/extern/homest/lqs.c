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

/* 
 * Least quantile of squares (LMedS for a quantile of 50%)
 * see Rousseeuw, Least Median of Squares Regression, Journal of the American
 * Statistics Association, Vol. 79, No. 388, pp. 871-880, Dec. 1984.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <float.h>

#include "lqs.h"
#include "compiler.h"

#define ROUND(x) ((int)((x) + 0.5))

#define PROBABILITY_CLOSE_TO_ONE 0.991       
/* seems to work in most cases. Only used when the number of sets is not supplied by the user */

/* If the number of combinations is less than MINIMUM_NUMBER_OF_CONFIGURATIONS
   times the number of sets needed to reach the probability of
   PROBABILITY_CLOSE_TO_ONE, then generate all possibilities */
#define MINIMUM_NUMBER_OF_CONFIGURATIONS 6


/*************************************************************************************************/
/* random number generators & distributions */
#ifdef WIN32
#include <process.h>
#define GETPID  _getpid
#else
#include <sys/types.h>
#include <unistd.h>
#define GETPID  getpid
#endif /* WIN32 */

#define USE_MARSAGLIA

#ifdef USE_MARSAGLIA
/* George Marsaglia's uniform random number generator */
#define S1NEW ((s1=36969*(s1&65535)+(s1>>16))<<16)
#define S2NEW ((s2=18000*(s2&65535)+(s2>>16))&65535)
#define RANDOM ((S1NEW+S2NEW)*2.32830643708e-10) /* 1/2^32 */
#define S1INIT  362436069
#define S2INIT  521288629
static unsigned long s1=S1INIT, s2=S2INIT;
#define SRANDOM(seed) s1=S1INIT, s2=S2INIT
#define REPEATABLE_NOISE_INIT_CNST 0 /* unused */

#else
#ifdef HAVE_RANDOM
/* random() returns numbers in the range from 0 to (2**31)-1 */
#define RANDOM ((double)random() / 2147483647.0)
#define SRANDOM(i) (srandom(i))
#else
/* rand() returns numbers in the range from 0 to (2**15)-1 */
#define RANDOM ((double)rand() / 32767.0)
#define SRANDOM(i) (srand(i))
#endif /* HAVE_RANDOM */

#define REPEATABLE_NOISE_INIT_CNST 42

#endif /* USE_MARSAGLIA */

#define UNIFORM_NOISE(low, high) ( (low) + RANDOM*((high) - (low)) )

/*
  set the flag specifying if random has to be initialized with a fixed seed. 
  If flag is set to 1 (default), several calls to the random routine
  will yield the same result, otherwise not
 */
static int _repeatablerandom_=1;

void lqs_setrepeatablerandom(int flag)
{
  _repeatablerandom_=flag;
}

/*
 * initialize the random sequence using pid as the seed 
 */
static int _randominitialized_ = 0;
void lqs_initnoise()
{
  if(_repeatablerandom_==1)
    SRANDOM(REPEATABLE_NOISE_INIT_CNST);
  else
	  if(_randominitialized_==0){
	    SRANDOM((int)GETPID());
	    _randominitialized_=1;
    }
}

/* this file uses the macro instead of lqs_uniformnoise() for speed */
double lqs_uniformnoise(double a, double b)
{
  return UNIFORM_NOISE(a, b);
}
/*************************************************************************************************/

/* this array of precomputed values corresponds to the inverse
   cumulative function for a normal distribution. For more information
   consult the literature (Robust Regression for Outlier Detection,
   rouseeuw-leroy) or the maple routine stats/statevalf[icdf,normald] 
   The values are computed with a step of 2.5% starting from 50% */
static double NICDF[21] = {
        1.4e16,      15.94723940, 7.957896558, 5.287692054, 
			  3.947153876, 3.138344200, 2.595242369, 2.203797543, 
			  1.906939402, 1.672911853, 1.482602218, 1.323775627, 
			  1.188182950, 1.069988721, 0.9648473415, 0.8693011162, 
			  0.7803041458, 0.6946704675, 0.6079568319, 0.5102134568,
			  0.3236002672};

#if 0
static double find_kpercentile(double *r, int n, int k)
{
register int i, ir, j, l, mid;
register double t;
double a;

  l = 0;
  ir = n-1;

  for (;;) {
    if (ir <= l + 1) {
      if ((ir == l + 1) && (r[ir] < r[l])){
        t=r[l]; r[l]=r[ir]; r[ir]=t;
      }
      return (r[k]);
    }
    else {
      mid = (l + ir) >> 1;
      t=r[mid]; r[mid]=r[l+1]; r[l+1]=t;
      if (r[l + 1] > r[ir]){
        t=r[l+1]; r[l+1]=r[ir]; r[ir]=t;
      }
      if (r[l] > r[ir]){
        t=r[l]; r[l]=r[ir]; r[ir]=t;
      }
      if (r[l + 1] > r[l]){
        t=r[l+1]; r[l+1]=r[l]; r[l]=t;
      }

      i = l + 1;
      j = ir;
      a = r[l];

      for (;;) {
        do
      	  i++;
        while (r[i] < a);
        do
          j--;
        while (r[j] > a);
        if (j < i)
      	  break;
        t=r[i]; r[i]=r[j]; r[j]=t;
      }

      r[l] = r[j];
      r[j] = a;

      if (j >= k)
        ir = j - 1;
      if (j <= k)
        l = i;
    }
  }
}
#endif

/*
 * Algorithm from N. Wirth's book, implementation by N. Devillard.
 * This code in public domain.
 * See http://ndevilla.free.fr/median/median/median.html
 */


/*---------------------------------------------------------------------------
   Function :   kth_smallest()
   In       :   array of elements, # of elements in the array, rank k
   Out      :   one element
   Job      :   find the kth smallest element in the array
   Notice   :   use the median() macro defined below to get the median. 

                Reference:

                  Author: Wirth, Niklaus 
                   Title: Algorithms + data structures = programs 
               Publisher: Englewood Cliffs: Prentice-Hall, 1976 
    Physical description: 366 p. 
                  Series: Prentice-Hall Series in Automatic Computation 

 ---------------------------------------------------------------------------*/


double kth_smallest(double *a, int n, int k)
{
register int i, j, l, m;
register double x, tmp;

    l=0; m=n-1;
    while(l<m){
        x=a[k];
        i=l;
        j=m;
        do{
            while(a[i]<x) ++i;
            while(x<a[j]) --j;
            if(i<=j){
                /* swap a[i], a[j] */
                tmp=a[i];
                a[i]=a[j];
                a[j]=tmp;
                ++i; --j;
            }
        } while(i<=j);
        if(j<k) l=i;
        if(k<i) m=j;
    }
    return a[k];
}

/******
#define median(a,n) kth_smallest(a, n, (((n)&1)? ((n)/2) :  (((n)/2)-1)) )
******/

/* sort routines for ints, avoiding the use of qsort */

static void shellsort_int(int *v, int n)
{
register int gap, i, j, jg;
register int tmp;

  for(gap=n/2; gap>0; gap>>=1) // gap/=2
    for(i=gap; i<n; i++)
      for(j=i-gap; j>=0 && v[j]>v[jg=(j+gap)]; j-=gap){
        tmp=v[j];
        v[j]=v[jg];
        v[jg]=tmp;
      }
}

inline static void insertionsort_int(int *v, int n)
{
register int i, j, tmp, *v1=v+1;

  for(i=0; i<n; ++i){
    for(j=i-1, tmp=v[i]; j>=0; --j){
      if(v[j]<=tmp) break;
      v1[j]=v[j];
    }
    v1[j]=tmp;
  }
}

/*
 * generate one subset making sure that all its elements are distinct
 */
static void lqs_gensubset(double h, int sizeSet, int subset[])
{
register int i, j;

    for(i=0; i<sizeSet; ++i){
resample:
      subset[i]=(int)UNIFORM_NOISE(0.0, h);
      for(j=0; j<i; j++) 
	      if(subset[i]==subset[j]) goto resample; /* element already in subset, try another one */
    }
}

/*
 * generate random subsets
 *
 * returns the number of random sets really generated
 */
static int lqs_genrandomsets(int sizeSet, int nbData, int nbSets, int **sets)
{
register int i, j, k;
double h;
int tries;
void (*howtosort)(int *v, int n);

    lqs_initnoise();

    h=nbData - 0.5;
    howtosort=(sizeSet<=20)? insertionsort_int : shellsort_int; /* insertion sort for sort lists, shell sort otherwise */

    for(i=tries=0; i<nbSets; ++i){
resample:
      lqs_gensubset(h, sizeSet, sets[i]);
      (*howtosort)(sets[i], sizeSet);

      /* check if set i is already contained in sets */
      for(j=0; j<i; ++j){
        for(k=0; k<sizeSet; ++k)
          if(sets[i][k]!=sets[j][k]) goto unequalsets; /* sets contain at least one different element */

        /* sets i, j are identical */
        if(++tries>3*nbSets/2){ /* premature termination, too many tries failed */
	        /* fprintf(stderr,"Warning: premature termination, too many tries failed in lqs_genrandomsets!\n");*/
          goto terminate;
	      }
        goto resample; /* discard set i */

unequalsets:
        continue; // not necessary but MSVC complains without it!
      }
    }

terminate:

    return i;
}

/*
 * generate combinations recursively
 */
static void lqs_gencombrec(int sizeSet, int ns[], int is[], int depth, int *nbComb, int **sets)
{
    if(depth==sizeSet-1){
      register int i;

      is[depth]=(depth==0)? 0 : is[depth-1]+1;
      for( ; is[depth]<ns[depth]; (is[depth])++){
        for(i=0; i<sizeSet; ++i)
		      sets[*nbComb][i]=is[i];
	        (*nbComb)++;
	      }
    }
    else{
      is[depth]=(depth==0)? 0 : is[depth-1]+1;
      for( ; is[depth]<ns[depth]; (is[depth])++)
	    lqs_gencombrec(sizeSet, ns, is, depth+1, nbComb, sets);
    }
}
    
/*
 * generate all combinations,
 * returns the number of combinations
 */
static int lqs_gencomb(
    int maxInt, /* maximum integer */
    int sizeSet, /* size of a combination */
    int **sets	/* Output: combinations generated */
)
{
int *is, *ns, nbComb;
register int i;

    if(!(is=(int *)malloc(2*sizeSet*sizeof(int)))){
	    fprintf(stderr, "Error: Not enough memory in `lqs_gencomb'!\n");
	    exit(1);
    }
    ns=is+sizeSet;

    for(i=0; i<sizeSet; ++i) 
      ns[i]=maxInt-sizeSet + i+1;

    nbComb=0;
    lqs_gencombrec(sizeSet, ns, is, 0, &nbComb, sets);

    free(is);

    return nbComb;
}

/*
 * number of combinations
 */
static int lqs_ncomb(int n, int m)
{
double nb;
int nbInt;
register int i;

  if(m<0 || m>n){
    fprintf(stderr, "\nError in argument m of `lqs_ncomb'!\n");
    return 0;
  }

  nb=1.0;
  for(i=0; i<m; ++i, --n)
    nb*=n;
  while(m>1)
    nb/=m--;
  nbInt=(nb>INT_MAX)? (INT_MAX) : ((int)nb);

  return nbInt;
}

/*
 * number of tries, i.e. m, according to
 * 1 - (1 - (1-e)^p)^m > P
 */
int lqs_numtries(
  int p,    /* I: size of a subsample */
  double i, /* I: expected fraction of inliers */
  double P  /* I: probability to have a good subsample */
)
{
double E;
register int j;

    if(i<0.0 || i>1.0 || p<=0 || P>=1.0 || P<=0.0){
      fprintf (stderr, "Error: invalid arguments to `lqs_numtries'!\n");
      exit(1);
    }

    for(j=1, E=i; j<p; ++j)
      E*=i;
    
    return (int) ceil(log(1.0 - P)/log(1.0 - E));
}

/*
 * allocate necessary subsets for LQS
 */
int **lqs_allocsets(
    int sizeSet, /* I: size of a subset */
		int nbSets	/* I: number of subsets */
)
{
int **sets;
register int i;

    if(!(sets=(int **)malloc(nbSets * sizeof(int *)))){
	    fprintf(stderr, "Error: Not enough memory in `lqs_allocsets'!\n");
	    exit(1);
    }

    if(!(sets[0]=(int *)malloc(sizeSet*nbSets*sizeof(int)))){
      fprintf(stderr, "Error: Not enough memory in `lqs_allocsets'!\n");
      exit(1);
    }

    for(i=1; i<nbSets; ++i)
      sets[i]=sets[i-1] + sizeSet;

    return sets;
}

/*
 * free allocated sets
 */
void lqs_freesets(int **sets)
{
    if(sets==NULL) return;

    free(sets[0]);
    free(sets);
}

static void dblcopy(double *dest, double *src, int n)
{
  while(--n>=0)
    *dest++=*src++;
}

/* 
 * Least quantile squares fitting technique
 *
 * returns 1 on success, 0 on failure
 */
int lqsfit(
  int nbData,       /* I: the number of Data, which is ordered from 0 */
  int sizeSet,      /* I: size of each randomly selected subset of data */
  int **sets,       /* I: randomly selected subsets, can be set to NULL forcing a default routine to be used for generating subsets */
  int nbSets,       /* I: the number of subsets, set to 0 if you have no idea */
  void (*residual)( /* I: function which calculates the residuals */
    double *x,        /* I: parameter vector */
    int nb,           /* I: number of residuals to be computed */
    double *resid),   /* O: computed residuals */
  int (*estimator)( /* I: function which estimate the parameters returning the number of solutions */
    double *x,        /* O: estimated parameter vector */
    int nb,           /* I: number of data to be used */
    int *indexes),    /* I: indexes of data to be used */
  int isResidualSqr,/* I: set 1 if `residual' computes the squared residuals */
  int verbose,      /* I: verbose mode */
  int maxNbSol,     /* I: maximum number of solutions given by "estimator" */
  double gate,      /* I: gate value for outlier detection, a value in [2, 4], often 2.5 */
  double prematureResidual, /* I: if > 0.0, LQS stops when a subset gives a quantile residual less than `prematureResidual'.
                                   * The specified value must be in accordance with `residual' */
  int dim,          /* I: dimension of the parameter vector */
  double percentageOfGoodData,  /* I: the percentage (between epsilon and 1.0) of good data. often 0.5 */
  double *estimate, /* O: the corresponding estimate of parameters */
  int *bestSet,     /* O: best subset retained, can be set to NULL */
  int *outliersMap, /* O: contains 1 in indexes of detected outliers, 0 in indexes of inliers */
  int *nbOutliers,  /* O: the number of detected outliers */
  double *outlierThresh /* O: residual threshold for determining outliers */
)
{
double *sols;
double *resGood, *resNew, *resTmp;
double quantMin=DBL_MAX, quantNew, sigma;
int quantPos, nbSol;
int freeSets=0;
int best=-1;
register int i, j, k;
int ret;

    /* memory allocation */
    if(!(sols=(double *)malloc(maxNbSol*dim*sizeof(double))) || !(resGood=(double *)malloc(3*nbData*sizeof(double)))){
      fprintf(stderr, "Error: Not enough memory in `lqsfit'!\n");
      exit(1);
    }

    resNew=resGood + nbData;
    resTmp=resNew + nbData;

    /* position of the quantile. If set to zero (meaningless) set it to 0.5 */
    if(percentageOfGoodData<=0.0 || percentageOfGoodData>=1.0){
	    fprintf(stderr,	"\nBad argument in lsqfit: percentageOfGoodData must be between 0.0 and 1.0\n");
      percentageOfGoodData=0.5;
    }
    quantPos=(int)(nbData*percentageOfGoodData); 
    if(quantPos<sizeSet - 1) quantPos=sizeSet - 1;

    /* if sets == NULL, use the default routine to generate random subsets */
    if(!sets){
	    int allComb;
	    freeSets=1;
	    allComb=lqs_ncomb(nbData, sizeSet);
	    if(nbSets==0)
	      nbSets=lqs_numtries(sizeSet, percentageOfGoodData, PROBABILITY_CLOSE_TO_ONE);
	    if(allComb<MINIMUM_NUMBER_OF_CONFIGURATIONS*nbSets){ /* generate all combinations */
	      sets=lqs_allocsets(sizeSet, allComb);
	      nbSets=lqs_gencomb(nbData, sizeSet, sets);
	    }
	    else{ /* randomly generate a few subsets */
	      sets=lqs_allocsets(sizeSet, nbSets);
	      nbSets=lqs_genrandomsets(sizeSet, nbData, nbSets, sets);
	    }
    }

    for(i=0; i<nbSets; ++i){
	    /* estimate the parameters for this subset */
	    nbSol=(*estimator)(sols, sizeSet, sets[i]);
	    /* now test the solutions on the whole set of data */
	    for(k=0; k<nbSol; ++k){
	      (*residual)(sols + k*dim, nbData, resNew);
	      /* we use the absolute residuals */
	      for(j=0; j<nbData; ++j)
		      resTmp[j]=resNew[j]=(resNew[j]>=0.0)? resNew[j] : -resNew[j]; /*fabs(resNew[j]);*/
        /*memcpy((char *)resTmp, (char *)resNew, nbData*sizeof(double));*/

        /* find the desired quantile of the set of residuals */
        /*quantNew=find_kpercentile(resTmp, nbData, quantPos);*/
        quantNew=kth_smallest(resTmp, nbData, quantPos);
        /* save the result */
        if(quantNew<quantMin){
          quantMin=quantNew;
          best=i;
          /*memcpy((char *)resGood, (char *)resNew, nbData*sizeof(double));*/
          /*memcpy((char *)estimate, (char *)(sols+k*dim), dim*sizeof(double));*/
          dblcopy(resGood, resNew, nbData);
          dblcopy(estimate, sols+k*dim, dim);

          if(quantMin<=prematureResidual)
            i=nbSets;	/* premature termination */
        }
	    }
    }

    if(best==-1){	/* failure, usually does not happen */
      fprintf(stderr, "LQS failed. Among all trials, no solution was found!\n");
      ret=0;
      goto cleanup;
    }

    if(verbose){
      printf("LQS result:\n Min quantile residual = %g\n", quantMin);
      printf(" Subset %d : ", best);
	    for(i=0; i<sizeSet; i++)
	      printf("%d ", sets[best][i]);
      printf("\n Estimate: ");
	    for(i=0; i<dim; i++)
	      printf("%g ", estimate[i]);
	    printf("\n");
      fflush(stdout);
    }

    if(bestSet)
	    *bestSet=best;

    if(gate<2.0 || gate>4.0){
	    fprintf(stderr,	"\nBad argument in lsqfit: gate must be between 2.0 and 4.0\n");
      gate=2.5;
    }

    /* compute the robust standard deviation estimate  */
    sigma=NICDF[(int)(ROUND(percentageOfGoodData*20.0))]*(1.0 + 5.0 / (double)(nbData - sizeSet));
    if(isResidualSqr)
	    sigma*=sigma;
    sigma*=gate*quantMin;
    if(isResidualSqr)
	    sigma*=gate;

    *outlierThresh=(isResidualSqr)? sqrt(sigma) : sigma;

    /* Now detect the outliers (use sqrt(KSQR) sigma) */
    for(i=j=0; i<nbData; ++i){
	    if(resGood[i]>sigma){ /* an outlier */
        outliersMap[i]=1;
	      if(verbose)
          printf(" Data %d is an outlier!\n", i);
        ++j;
      }
      else
        outliersMap[i]=0;
    }
    *nbOutliers=j;
    ret=1; /* success */

cleanup:
    /* free memory */
    free(sols);
    free(resGood);
    if(freeSets)
	    lqs_freesets(sets);

    return ret;
}

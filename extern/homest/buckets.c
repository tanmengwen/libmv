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
#include "lqs.h"

#undef USE_UNIQUE_SETS

//#define USE_UNIQUE_SETS

#define NUM_X_BUCKETS 5
#define NUM_Y_BUCKETS 5

#define MAX_TRIES       60

struct bucket{
  int nummatches;	            /* number of matches in bucket	*/
  int *indxs;                 /* index into points array */
};

struct selection{
  struct bucket *buck;        /* points to corresponding bucket */
  int selflag;	              /* indicates whether the corresponding bucket has been selected */
  double interval_width;	    /* width of the corresponding interval in [0, 1] */
};

static struct bucket buckets[NUM_X_BUCKETS][NUM_Y_BUCKETS];

static void shellsort_int(int *v, int n)
{
  register int gap, i, j, jg;
  register int tmp;

  for(gap=n/2; gap>0; gap/=2)
    for(i=gap; i<n; i++)
      for(j=i-gap; j>=0 && v[j]>v[jg=(j+gap)]; j-=gap){
        tmp=v[j];
        v[j]=v[jg];
        v[jg]=tmp;
      }
}

/* Sets generation without buckets */

/*
 * generate one subset
 */
static void genOneSubsetNoBuckets(double h, int sizeSet, int subset[])
{
register int i, j, in;

  for(i=0; i<sizeSet;  ){
    subset[i]=(int)lqs_uniformnoise(0.0, h);
    in=0;
    for(j=0; j<i && in==0; j++) 
      if(subset[i]==subset[j]) in=1;
    if(in==0) i++;
  }
}

/*
 * generate random subsets without using buckets
 *
 * returns the number of random sets actually generated
 */
int genRandomSetsNoBuckets(int sizeSet, int nbData, int nbSets, int **sets)
{
  register int i, j, k;
  int *subset;
  double l, h;
  int tries;
#ifdef USE_UNIQUE_SETS
  int in, out;
#endif /* USE_UNIQUE_SETS */

  lqs_initnoise();

  l=0.0;
  h=nbData - 0.5;
  tries=0;
  for(i=0; i<nbSets; ){
      /* generate a subset */
      subset=sets[i];
      /* following code adapted from genOneSubsetNoBuckets() which
         is identical to lqs_gensubset()
       */
      for(j=0; j<sizeSet; j++){
resample:
          subset[j]=(int)lqs_uniformnoise(l, h);
          for(k=0; k<j; k++)
	          if(subset[j]==subset[k]) goto resample; /* element already in subset, try another one */
      }

#ifdef USE_UNIQUE_SETS
      shellsort_int(sets[i], sizeSet);
      /* verify whether the subset is already retained in sets */
      in=0;
      for(j=0; j<i && in==0; j++){
	      out=0;
	      for(k=0; k<sizeSet && out==0; k++)
	        if(sets[i][k]!=sets[j][k])
	          out=1;
	      if(out==0)
	        in=1;
	    }
      if(in==0)
	      i++;
#else
	      i++;
#endif /* USE_UNIQUE_SETS */

      /* premature exit, too many tries failed */
      if(++tries > 3 * nbSets / 2){
	  /*fprintf (stderr,"Warning: premature return in genRandomSetsNoBuckets(), too many tries failed!\n"); */
	      break;
	    }
  }

  return i;
}

/* Sets generation with buckets */

/* For more details on buckets, see INRIA RR-2273, pp. 19 */

/* initializes buckets and returns 1 if enough nonempty buckets exist, 0 otherwise */
static int initBuckets(double (*pts)[2], int npts, int sizeSet)
{
register int i, bx, by;
register struct bucket *bptr;
int numnonempty;
double coord, maxu, minu, maxv, minv, bucket_width, bucket_height;

	/* compute min-max corner coordinates */
	maxu=maxv=DBL_MIN;
	minu=minv=DBL_MAX;

	for(i=0; i<npts; i++){
    coord=pts[i][0];
    if(coord > maxu) maxu=coord;
    if(coord < minu) minu=coord;

    coord=pts[i][1];
    if(coord > maxv) maxv=coord;
    if(coord < minv) minv=coord;
	}

  maxu++; maxv++;
  minu--; minv--;

	/* init bucket array */
	for(bx=0; bx<NUM_X_BUCKETS; bx++)
		for(by=0; by<NUM_Y_BUCKETS; by++){
			buckets[bx][by].nummatches=0;
			if(!(buckets[bx][by].indxs=(int *)malloc(npts*sizeof(int)))){
        fprintf(stderr, "Memory allocation request failed in initBuckets()!\n");
        exit(1);
      }
		}

	/* count points in buckets */
	bucket_width=(maxu-minu)/(double)(NUM_X_BUCKETS);
	bucket_height=(maxv-minv)/(double)(NUM_Y_BUCKETS);

	for(i=0; i<npts; i++){
    coord=pts[i][0];
		bx=(int)((coord-minu)/bucket_width);

    coord=pts[i][1];
		by=(int)((coord-minv)/bucket_height);

    bptr=buckets[bx] + by;

    bptr->indxs[bptr->nummatches]=i;
    (bptr->nummatches)++;
    //printf("POINT %d goes to bucket %d %d\n", i, bx, by);
	}

	/* count nonempty buckets */
  for(i=numnonempty=0, bptr=buckets[0]; i<NUM_X_BUCKETS*NUM_Y_BUCKETS; i++, bptr++){
      if(bptr->nummatches==0) continue;

			numnonempty++;
  }

	if(numnonempty<sizeSet){
		fprintf(stderr, "Warning: Number of nonempty buckets in initBuckets() (%d) is smaller than required (%d)!\n%s\n",
						numnonempty, sizeSet, "Buckets will not be used; to enable them reduce their number");
		return 0;
	}

  return 1;
}

/* generate a random subset using buckets. No check is performed for 
 * avoiding cases where a point is contained more than once in the
 * computed subset
 */
static void genOneSetWithBuckets(double nbData, int sizeSet, int subset[])
{
register int i, j, tries;
register struct bucket *bptr;
int     nselected;
double 	anum, sum;
struct  selection selected_bucks[NUM_X_BUCKETS*NUM_Y_BUCKETS];

	/* prepare to start selecting buckets */
  for(i=j=0, bptr=buckets[0]; i<NUM_X_BUCKETS*NUM_Y_BUCKETS; i++, bptr++){
			if(bptr->nummatches==0) continue;

			selected_bucks[j].buck=bptr;
			selected_bucks[j].interval_width=((double)(bptr->nummatches))/nbData;
			selected_bucks[j].selflag=0;
			j++;
		}

	nselected=0;
	while(nselected<sizeSet){
		tries=0;
    do{
		  anum=lqs_uniformnoise(0.0, 0.999);
      tries++;

		  i=-1; sum=0.0;
		  do{
			  i++;
			  sum+=selected_bucks[i].interval_width;
		  } while(sum<anum);

    } while(selected_bucks[i].selflag && tries<MAX_TRIES);

    /* bucket `i' is either not selected or the `tries' limit has been reached */

		selected_bucks[i].selflag=1;

		/* choose a point from the selected bucket */
    j=(int)lqs_uniformnoise(0.0, (double)((selected_bucks[i].buck)->nummatches-0.5));
    subset[nselected]=(selected_bucks[i].buck)->indxs[j];
		nselected++;
	}
}

/*
 * generate random subsets using buckets
 *
 * returns the number of random sets actually generated
 */
int genRandomSetsWithBuckets(double (*pts)[2], int sizeSet, int nbData, int nbSets, int **sets)
{
  register int i, j;
  int *subset, ret;
  int tries;
  void (*func)(double, int, int*);

#ifdef USE_UNIQUE_SETS
  register int k;
  int in, out;
#endif /* USE_UNIQUE_SETS */

  lqs_initnoise();

  if(initBuckets(pts, nbData, sizeSet))
    func=genOneSetWithBuckets;
  else
    func=genOneSubsetNoBuckets;
  
  tries=0;
  for(i=0; i<nbSets; ){
      /* generate a subset */
      subset=sets[i];
      (*func)(nbData, sizeSet, subset);

#ifdef USE_UNIQUE_SETS
      shellsort_int(sets[i], sizeSet);
      /* verify whether the subset is already contained in sets */
      in=0;
      for(j=0; j<i && in==0; j++){
	      out=0;
	      for(k=0; k<sizeSet && out==0; k++)
	        if(sets[i][k]!=sets[j][k])
	          out=1;
	      if(out==0)
	        in=1;
	    }
      if(in==0)
	      i++;
#else
	      i++;
#endif /* USE_UNIQUE_SETS */

      /* premature exit, too many tries failed */
      if(++tries > 3*nbSets/2){
        /*fprintf (stderr,"Warning: premature return in genRandomSetsWithBuckets(), too many tries failed!\n"); */
	      break;
	    }
  }
  ret=i;

  /* free bucket indices */
	for(i=0; i<NUM_X_BUCKETS; i++)
		for(j=0; j<NUM_Y_BUCKETS; j++)
      free(buckets[i][j].indxs);

  return ret;
}

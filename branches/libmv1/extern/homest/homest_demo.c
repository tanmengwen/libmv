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

/******************************************************************************** 
 * homest demo. The program accepts a text file containing quadruples of matching
 * point coordinates (i.e., x1 y1  x2 y2 where x1 y1 is a (corner) point in the
 * 1st image and x2 y2 its corresponding one in the 2nd) and estimates the
 * homography mapping points in the first image to points in the second
 ********************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "homest.h"

#define MAXSTRLEN	256

/* read pairs of matching points from a file */
int read2DMatches(char *fname, double (**pts0)[2], double (**pts1)[2])
{
register int i;
int ncoords, nmatches;
double coords[4];
FILE *fp;
char buf[MAXSTRLEN];

  if((fp=fopen(fname, "r"))==NULL){
    fprintf(stderr, "cannot open file %s\n", fname);
    exit(1);
  }

  fgets(buf, MAXSTRLEN, fp);
  if(ferror(fp)){
    fprintf(stderr, "File %s: error reading first line\n", fname);
    exit(1);
  }

  ncoords=sscanf(buf, "%lf%lf%lf%lf", coords, coords+1, coords+2, coords+3);
  if(ncoords==4){ /* no lines number */
    for(nmatches=1; !feof(fp); nmatches++){
      fscanf(fp, "%*g%*g%*g%*g\n");
      if(ferror(fp)){
        fprintf(stderr, "File %s: error reading 2D point coordinates, line %d\n", fname, nmatches + 1);
        exit(1);
      }
    }

    rewind(fp);
  }
  else{
    sscanf(buf, "%d", &nmatches);
  }

  *pts0=(double (*)[2])malloc(nmatches*sizeof(double[2]));
  *pts1=(double (*)[2])malloc(nmatches*sizeof(double[2]));
  if(!*pts0 || !*pts1){
    fprintf(stderr, "Memory allocation request failed in read2DMatches()\n");
    exit(1);
  }

  /* read in points and store them */
  for(i=0; !feof(fp); i++){
    ncoords=fscanf(fp, "%lf%lf%lf%lf\n", (*pts0)[i], (*pts0)[i]+1, (*pts1)[i], (*pts1)[i]+1);
    if(ncoords==EOF) break;

    if(ncoords!=4){
      fprintf(stderr, "File %s: line %d contains only %d coordinates\n", fname, i + 1, ncoords);
      exit(1);
    }

    if(ferror(fp)){
      fprintf(stderr, "File %s: error reading 2D point coordinates, line %d\n", fname, i + 1);
      exit(1);
    }

  }
  fclose(fp);

  if(i!=nmatches){
    fprintf(stderr, "number of actuall points in file %s does not agree with that in first line (%d != %d)!\n",
                     fname, i, nmatches);
    exit(1);
  }

  return nmatches;
}

#define INL_PCENT 0.7

int main(int argc, char *argv[])
{
double (*pts0)[2], (*pts1)[2];
register int i;
int npts, noutl, cfunc;
double H[NUM_HPARAMS], rms, rmeds;

  if(argc!=2){
    fprintf(stderr, "Usage: %s <matched points>\n", argv[0]);
    exit(1);
  }

  npts=read2DMatches(argv[1], &pts0, &pts1);

#if 0
  for(i=0; i<npts; ++i){
    printf("%g %g  %g %g\n", pts0[i][0], pts0[i][1], pts1[i][0], pts1[i][1]);
  }
#endif

  cfunc=HOMEST_SYM_XFER_ERROR; // use the symmetric transfer error
  //cfunc=HOMEST_SAMPSON_ERROR; //use the sampson error
  homest(pts0, pts1, npts, INL_PCENT, H, 1, cfunc, &noutl, 1);

  printf("%s: estimated homography [%d outliers out of %d matches]\n", argv[0], noutl, npts);
  for(i=0; i<NUM_HPARAMS; ++i){
    if(!(i%3)) printf("\n");
    printf("%.7g ", H[i]);
  }
  printf("\n");

  homest_RMS_RMedS(pts0, pts1, npts, H, &rms, &rmeds);
  printf("\nHomography RMS and RMedS errors for input points: %g %g\n", rms, rmeds);

  free(pts0);
  free(pts1);

  return 0;
}

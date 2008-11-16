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

#ifndef _HOMEST_H
#define _HOMEST_H

//#define DLL_BUILD /* define this if you want to build a DLL with MSVC */

#ifdef __cplusplus
extern "C" {
#endif

#define HOMEST_VERSION    "1.0 (Jul. 2006)"

#define NUM_HPARAMS       9

/* non-linear refinement cost functions */
#define HOMEST_NO_NLN_REFINE    0 /* no non-linear refinement */
#define HOMEST_SYM_XFER_ERROR   1 /* non-linear refinement using symmetric homographic transfer error */
#define HOMEST_SAMPSON_ERROR    2 /* non-linear refinement using Sampson error */

#if defined(DLL_BUILD) && defined(_MSC_VER) /* build DLLs with MSVC only! */
#define HOMEST_API_MOD    __declspec(dllexport)
#define HOMEST_CALL_CONV  __cdecl
#else /* define empty */
#define HOMEST_API_MOD 
#define HOMEST_CALL_CONV
#endif /* DLL_BUILD && _MSC_VER */

/* homest.c */
extern HOMEST_API_MOD void HOMEST_CALL_CONV
  homest(double (*pts0)[2], double (*pts1)[2], int nmatches, double inlPcent, double H01[NUM_HPARAMS],
          int normalize, int NLrefine, int *nbOutliers, int verbose);

extern HOMEST_API_MOD void HOMEST_CALL_CONV
  homest_RMS_RMedS(double (*inpts0)[2], double (*inpts1)[2], int nmatches, double H[NUM_HPARAMS],
                    double *rms, double *rmeds);

#ifdef __cplusplus
}
#endif

#endif /* _HOMEST_H */

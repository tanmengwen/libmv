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

#ifndef _MAPLE_FUNCS_H
#define _MAPLE_FUNCS_H

/* calc_2Dhomog_coeffs.c */
extern void calc2DHomogLinCoeffs2(double m1[2], double m2[2], double eq[18]);
extern void calc2DHomogLinCoeffs3(double m1[2], double m2[2], double eq[27]);
extern void calc2DHomogNonLinErr(double m1[2], double m2[2], double h[9], double *err);
extern void calc2DHomogNonLinErrGrads(double m1[2], double m2[2], double h[9], double grads[9]);
extern void calc2DHomogSampsonErr(double m1[2], double m2[2], double h[9], double *err);
extern void calc2DHomogSampsonErrGrads(double m1[2], double m2[2], double h[9], double grads[9]);

#endif /* _MAPLE_FUNCS_H */

#################################################################################
# 
#  Non-linear, robust homography estimation
#  Copyright (C) 2003-06  Manolis Lourakis (lourakis@ics.forth.gr)
#  Institute of Computer Science, Foundation for Research & Technology - Hellas
#  Heraklion, Crete, Greece.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#################################################################################

# 2D homography estimation

with(linalg):
with(codegen):

# 9x1 vector converted to 3x3 matrix
vec2mat3x3:=proc(a::array(1..9), H::array(1..3, 1..3))
    H:=array(1..3, 1..3, [[a[1], a[2], a[3]], 
                          [a[4], a[5], a[6]],
                          [a[7], a[8], a[9]]]);
end:

# H p 
Mk2DHomogExpr:=proc(p1::array(1..3), a::array(1..9), q2::array(1..3))

local H;

    H:=array(1..3, 1..3);
    vec2mat3x3(a, H);

    q2:=array(1..3); 
    q2:=evalm(H&*p1);
end:

# Two equations per point. This assumes that the 3rd coordinate is nonzero, which is
# always true for image corners. Hence, this is suitable for estimating a homography
# from point correspondences
Mk2DHomogEqsLin2:=proc(p1::array(1..3), p2::array(1..3), a::array(1..9), eqs::array(1..2))
local q2;

    q2:=array(1..3);
    Mk2DHomogExpr(p1, a, q2);

    eqs[1]:=simplify(p2[1]*q2[3] - p2[3]*q2[1]);
    eqs[2]:=simplify(p2[2]*q2[3] - p2[3]*q2[2]);
end:

# Three equations per point. This makes no assumption about nonzero coordinates but generates
# larger linear systems. This is suitable for estimating a homography from line or
# ideal point correspondences
Mk2DHomogEqsLin3:=proc(p1::array(1..3), p2::array(1..3), a::array(1..9), eqs::array(1..3))
local q2, cp;

    q2:=array(1..3);
    Mk2DHomogExpr(p1, a, q2);

    cp:=crossprod(p2, q2); # the crossproduct must be zero
    eqs[1]:=cp[1];
    eqs[2]:=cp[2];
    eqs[3]:=cp[3];
end:

Mk2DHomogEqsNonLin2:=proc(p1::array(1..3), p2::array(1..3), a::array(1..9), err)
local q2, aH;

    q2:=array(1..3);
    Mk2DHomogExpr(p1, a, q2);

    aH:=array(1..3, 1..3);
    vec2mat3x3(a, aH);
    aHinv:=inverse(aH);
    q1:=array(1..3);
    q1:=evalm(aHinv&*p2);

    err:=sqrt((p2[1]/p2[3] - q2[1]/q2[3])^2 + (p2[2]/p2[3] - q2[2]/q2[3])^2 +
         (q1[1]/q1[3] - p1[1]/p1[3])^2 + (q1[2]/q1[3] - p1[2]/p1[3])^2);
end:

# Sampson approximation to geometric error for a homography, see HZ2 p. 99
Mk2DHomogEqsSampson2:=proc(parms::array(1..4), a::array(1..9), e::array(1..2), err)
local tmp, i, j, Jac;

  tmp:=array(1..4);
  Jac:=array(1..2, 1..4);

  for i from 1 to 2 do
    tmp:=grad(e[i], parms);
    for j from 1 to 4 do
      Jac[i, j]:=tmp[j];
    end do:
  end do:

  err:=sqrt(evalm(transpose(e)&*inverse(Jac&*transpose(Jac))&*e));
end:


m1:=array(1..2);
m2:=array(1..2);
p1:=array(1..3, [m1[1], m1[2], 1]):
p2:=array(1..3, [m2[1], m2[2], 1]):
meas:=array(1..4, [m1[1], m1[2], m2[1], m2[2]]):
h:=array(1..9):
eqs2:=array(1..2):
eqs3:=array(1..3):

Mk2DHomogEqsLin2(p1, p2, h, eqs2):
Mk2DHomogEqsLin3(p1, p2, h, eqs3):
Mk2DHomogEqsSampson2(meas, h, eqs2, samperr): 

vars:=vector([h[1], h[2], h[3], 
              h[4], h[5], h[6],
              h[7], h[8], h[9]]):
Mk2DHomogEqsNonLin2(p1, p2, h, nlerr):
nlerr_gradient:=grad(nlerr, vars):
samperr_gradient:=grad(samperr, vars):

calc2DHomogLinCoeffs2:=optimize(makeproc([
        seq(eq[j]  =coeff(eqs2[1], vars[j], 1), j=1..9),\
        seq(eq[9+j]=coeff(eqs2[2], vars[j], 1), j=1..9)],\
        parameters=[m1::array(1..2), m2::array(1..2),\
        eq::array(1..2*9)])):
calc2DHomogLinCoeffs2:=makevoid(calc2DHomogLinCoeffs2):


calc2DHomogLinCoeffs3:=optimize(makeproc([
        seq(eq[j]    =coeff(eqs3[1], vars[j], 1), j=1..9),\
        seq(eq[9+j]  =coeff(eqs3[2], vars[j], 1), j=1..9),\
        seq(eq[2*9+j]=coeff(eqs3[3], vars[j], 1), j=1..9)],\
        parameters=[m1::array(1..2), m2::array(1..2),\
        eq::array(1..3*9)])):
calc2DHomogLinCoeffs3:=makevoid(calc2DHomogLinCoeffs3):


calc2DHomogNonLinErr:=optimize(makeproc([
                          err[1]=nlerr],\
                          parameters=[m1::array(1..2), m2::array(1..2),\
                          h::array(1..9), err::array(1..1)])):
calc2DHomogNonLinErr:=makevoid(calc2DHomogNonLinErr):

calc2DHomogNonLinErrGrads:=optimize(makeproc([
                          seq(grads[i]=nlerr_gradient[i], i=1..9)],\
                          parameters=[m1::array(1..2), m2::array(1..2),\
                          h::array(1..9), grads::array(1..9)])):
calc2DHomogNonLinErrGrads:=makevoid(calc2DHomogNonLinErrGrads):

calc2DHomogSampsonErr:=optimize(makeproc([
                          err[1]=samperr],\
                          parameters=[m1::array(1..2), m2::array(1..2),\
                          h::array(1..9), err::array(1..1)])):
calc2DHomogSampsonErr:=makevoid(calc2DHomogSampsonErr):

calc2DHomogSampsonErrGrads:=optimize(makeproc([
                          seq(grads[i]=samperr_gradient[i], i=1..9)],\
                          parameters=[m1::array(1..2), m2::array(1..2),\
                          h::array(1..9), grads::array(1..9)])):
calc2DHomogSampsonErrGrads:=makevoid(calc2DHomogSampsonErrGrads):

# Generate C code
fname:="./calc_2Dhomog_coeffs.c";
fd:=open(fname, WRITE);
fprintf(fd, "/* Code automatically generated by maple */\n\n");
fprintf(fd, "#include \"maplefuncs.h\"\n\n");
close(fd);

C(calc2DHomogLinCoeffs2, declarations='[m1::array(1..2), m2::array(1..2),\
                             eq::array(1..2*9)]', filename=fname):

C(calc2DHomogLinCoeffs3, declarations='[m1::array(1..2), m2::array(1..2),\
                             eq::array(1..3*9)]', filename=fname):

C(calc2DHomogNonLinErr, declarations='[m1::array(1..2), m2::array(1..2), h::array(1..9),\
                                  err::array(1..1)]', filename=fname):

C(calc2DHomogNonLinErrGrads, declarations='[m1::array(1..2), m2::array(1..2), h::array(1..9),\
                             grads::array(1..9)]',
                             filename=fname):

C(calc2DHomogSampsonErr, declarations='[m1::array(1..2), m2::array(1..2), h::array(1..9),\
                                  err::array(1..1)]', filename=fname):

C(calc2DHomogSampsonErrGrads, declarations='[m1::array(1..2), m2::array(1..2), h::array(1..9),\
                             grads::array(1..9)]',
                             filename=fname):

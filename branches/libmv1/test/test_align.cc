#include "testmv.h"
#include "mv.h"
#include "align.h"

static double z[] = { // zero mean
 -0.59872093,  0.22487961,  1.61976639, -0.48354586, -0.76237921,
 -0.56162201, -0.65732084,  1.87194361,  1.31376597, -1.96676673,
  1.21398488, -0.0651134 , -0.04198415, -0.93630598, -0.17058135
};

static double w[] = { // zero mean, average mag 1.0 (3x5 not 5x3 confusingly)
 -0.35471443479704967, 0.13323075884570362, 0.95963660130565653,
 -0.28647853649989047,-0.45167438885441996,-0.33273504021742589,
 -0.3894321688068626 , 1.10903993057233086, 0.77834551598762169,
 -1.16521823753566367, 0.71922984204431162,-0.03857667523025335,
 -0.02487366724345025,-0.55471794548064812,-0.10106155408995988
};

static double r[] = { // rotation matrix
 -0.48873319723957276, 0.80329650029378652, 0.34037419780550249,
  0.10566828612742996,-0.33276733687102406, 0.93706996153897593,
  0.86601043597804084, 0.49394395646421363, 0.07775148004766663
};

static double r_dot_w[] = { // r*w
  0.15088310701723034,-0.39107429793118792, 0.41341527603611627,
  0.57644212439503106,-0.74966620951718943, 0.74720996724930844,
  0.10751952807813411,-0.29095747562221264,-0.80908918432147248,
  0.24531716461624237,-0.41561767983217718,-0.07997783227571857,
  1.37692491821849949, 0.09323552010036323,-0.97456492621096669
};

// Euclidean alignment
TEST(find_aligning_transform_identity) {
	mat X(3,5);
	fill(X,z);
	mat T;
	find_aligning_transform(X,X,&T);
	mat residuals = T - eye(4);
	AllVerySmall(residuals);
}

TEST(find_aligning_transform_scale) {
	mat X(3,5), Y(3,5);
	fill(X,z);
	fill(Y,z);
	Y *= 10.0;
	mat T;
	find_aligning_transform(X,Y,&T);
	mat expected = eye(4)/10.0;
	expected(3,3) = 1.0;
	mat residuals = T - expected;
	AllVerySmall(residuals);
}

TEST(find_aligning_transform_shift) {
	mat X(3,5), Y(3,5);
	fill(X,w);
	fill(Y,w);
	for (int i=0; i<3; i++)
		for (int j=0; j<5; j++)
			Y(i,j) += 10;
	mat T;
	find_aligning_transform(X,Y,&T);
	mat residuals = eye(4);
	residuals(0,3) = -10;
	residuals(1,3) = -10;
	residuals(2,3) = -10;
	residuals -= T;
	AllVerySmall(residuals);
}

TEST(find_aligning_transform_rotation) {
	mat X(3,5), Y(3,5);
	fill(X,w);
	fill(Y,r_dot_w);
	mat T;
	find_aligning_transform(X,Y,&T);
	mat R(3,3);
	fill(R, r);
	mat residuals = eye(4);
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			residuals(i,j) = R(j,i); // transpose!
	residuals -= T;
	AllVerySmall(residuals);
}

// Projective alignment
#include "test_align_data.h"

TEST(find_aligning_projective_transform) {
	MATDATA(X);
	MATDATA(Xp);
	MATDATA(T);
	mat T_est(4,4);
	find_aligning_projective_transform(Xp,X,&T_est);
	mat resid = T - T_est;
	AllVerySmall(resid);
}

TEST(find_aligning_projective_transform_trivial_identity) {
	MATDATA(X);
	mat T_est(4,4);
	find_aligning_projective_transform(X,X,&T_est);
	for (int i=0; i<4; i++)
		VeryClose(T_est(i,i), 0.5);
}

// each column of xpp is not scaled by a different amount like
// with the first test
TEST(find_aligning_projective_transform_no_scalar_constant) {
	MATDATA(X);
	MATDATA(Xpp);
	MATDATA(T);
	mat T_est(4,4);
	find_aligning_projective_transform(Xpp,X,&T_est);
	mat resid = T - T_est;
	AllVerySmall(resid);
}

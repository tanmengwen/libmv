#include "testmv.h"

TEST() {
	vec x(3);
	x[0] = 1;
	x[1] = 0;
	x[2] = 0;

	mat y(crossmat(x));
	vec z = prod(y,x);
	VerySmall(z[0]);
	VerySmall(z[1]);
	VerySmall(z[2]);

	x[0] = 0;
	x[1] = 1;
	x[2] = 0;
	z = prod(y,x);
	VeryClose(z[0], 0.);
	VeryClose(z[1], 0.);
	VeryClose(z[2], 1.);

	y = crossmat(x);
	z = prod(y,x);
	VerySmall(z[0]);
	VerySmall(z[1]);
	VerySmall(z[2]);
}

TEST() {
	mat x(3,2);
	x(0,0) = 1.0; x(0,1) = 0.0; 
	x(1,0) = 2.0; x(1,1) = 0.0; 
	x(2,0) = 0.0; x(2,1) = 0.0; 

	vec v;
	double s = nullspace(x, v);

	VerySmall(s);

	VeryClose(v[0], 0.);
	VeryClose(v[1], 1.);
}

TEST() {
	mat x(2,3);
	x(0,0) = 1.0; x(0,1) = 0.0; x(0,2) = 0.0;
	x(1,0) = 2.0; x(1,1) = 1.0; x(1,2) = 1.0;

	vec v;
	double s = nullspace(x, v);

	Zero(s);
}

TEST() {
	mat x(2,4);
	x(0,0) = 1.0; x(0,1) = 0.0; x(0,2) = 0.0; x(0,3) = 1.0; 
	x(1,0) = 2.0; x(1,1) = 1.0; x(1,2) = 1.0; x(1,3) = 1.0; 

	vec a,b;
	double s = nullspace(x, a,b);
	double aa[] = {-0.62047572,  0.4472136 ,  0.17326212,  0.62047572};
	double bb[] = {-0.12251482, -0.63245553,  0.75497035,  0.12251482};
	for (int i=0; i<4; i++) {
		Close(a[i],aa[i]);
		Close(b[i],bb[i]);
	}

	Zero(s);
}

// not exactly thorough but hey
TEST() {
	mat x = eye(3,3);
	mat z = inv(x);
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++) {
			VeryClose(z(i,j), x(i,j));
		}
}


double x[] = {
         0.12086345,  0.60982246,  0.88273748,  0.26574395,
         0.78798681,  0.3865671 ,  0.61244456,  0.26715557,
         0.0478559 ,  0.34242683,  0.96801226,  0.26963634};

double pinv_x[] = {
        -0.7686931 ,  1.38260358, -0.20338537,
         3.57776493, -0.36927677, -3.03782066,
        -1.15408464,  0.01861476,  2.00921439,
        -0.26394109,  0.15674897,  0.38948875};

TEST() {
	mat X(3,4);
	fill(X, x);

	mat Z(4,3);
	fill(Z, pinv_x);

	mat Y = pinv(X);
	for (int i=0; i<4; i++)
		for (int j=0; j<3; j++)
			Close(Z(i,j), Y(i,j));
}

double y[] = {
         0.35065471,  0.03524406,  0.07236074,
         0.53306534,  0.06427368,  0.09703774,
         0.87950448,  0.50538372,  0.00960493,
         0.93742079,  0.77708692,  0.42075162};

double pinv_y[] = {
         0.88843765,  1.30574617,  0.48705807, -0.46505542,
        -1.61489908, -2.26172984,  1.1916875 ,  0.77214765,
         1.02189336,  1.25494216, -3.28538284,  1.98652627};

TEST() {
	mat X(4,3);
	fill(X, y);

	mat Z(3,4);
	fill(Z, pinv_y);

	mat Y = pinv(X);
	for (int i=0; i<3; i++)
		for (int j=0; j<4; j++)
			Close(Z(i,j), Y(i,j));
}

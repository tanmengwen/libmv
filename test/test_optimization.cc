#include "testmv.h"

TEST(points_put_on_disk_are_in_2pi_radius) {
	vec P(4);
	P[0] = 1;
	P[1] = 1;
	P[2] = 1;
	P[3] = 1;
	vec R(sphere_to_disk(P));
	True(norm_2(R), 2*M_PI, norm_2(R) < 2*M_PI);
}

TEST(can_get_from_disk_to_sphere_and_back) {
	vec R(4);
	R[0] = 1;
	R[1] = 1;
	R[2] = 1;
	R[3] = 1;

	vec P(disk_to_sphere(R));
	vec R2(sphere_to_disk(P));
	vec resids = R2-R;
	AllVerySmall(resids);
}

TEST(can_get_from_disk_to_sphere_and_back) {
	double r[]={ 0.799443 ,  0.0799443,  2.39833};
	vec R(3);
	fill(R,r);
	double p[]={ 0.30137428, 0.03013743, 0.90412323, 0.30137429};
	vec P(4);
	fill(P,p);
	P /= norm_2(P);

	vec resids1 = P-disk_to_sphere(R);
	vec resids2 = R-sphere_to_disk(P);
	AllSmall(resids1);
	AllSmall(resids2);
}

TEST(can_get_from_sphere_to_disk_and_back) {
	vec P(4);
	P[0] = 0.1;
	P[1] = 0.01;
	P[2] = 0.3;
	P[3] = 0.1;
	P /= norm_2(P);

	vec R(sphere_to_disk(P));
	vec P2(disk_to_sphere(R));
	vec resids = P2-P;
	AllVerySmall(resids);
}

TEST(jacobian_sphere_disk) {
	vec X(2);
	X[0] = 0.5;
	X[1] = 0.2;
	mat J(jacobian_sphere_disk(X));

	// these were generated from vispy, which has a gradchecker
	Close(J(0,0),  4.83638855e-01);
	Close(J(0,1), -4.13653645e-03);

	Close(J(1,0), -4.13653645e-03);
	Close(J(1,1),  4.92325582e-01);

	Close(J(2,0), -1.23495049e-01);
	Close(J(2,1), -4.93980197e-02);
}

TEST(check_reprojection_error_matches_from_real_case) {
	// These were pulled from the loop in bundle() where we find visibility
	double ss_[] = {-0.597857,0.0810615,0.559678,-0.568116,};
	vec ss(4); fill(ss,ss_);
	double f_[] = {410.69,218.226,1,};
	vec f(3); fill(f,f_);
	double P_[] = {-37.1907,60.3509,-27.2546,67.8507,-121.5,34.5913,-21.0076,137.212,-0.598052,0.120059,-0.0428123,0.718784,};
	mat P(3,4); fill(P,P_);

	double xij[2];

	vec p = sphere_to_disk(ravel(P));
	double p_[11];
	for (int i=0; i<11; i++)
		p_[i]=p[i];

	vec x;
	x= sphere_to_disk(ss);
	double x_[3];
	for (int i=0; i<3; i++)
		x_[i]=x[i];

	sba_proj_projective(0,0,(double*)p_,(double*)x_,(double*)xij,(void*)NULL);

	vec err(2);
	err[0] = xij[0]-f[0];
	err[1] = xij[1]-f[1];
//	printf("ERRORX=%g\n",xij[0]-f[0]);
//	printf("ERRORX=%g\n",xij[1]-f[1]);
//	printf("||ERROR||=%g\n",norm_2(err));

	vec xxx = dot(P,ss);
	xxx /= xxx[2];
	err[0] = xxx[0]-f[0];
	err[1] = xxx[1]-f[1];
	double er = norm_2(err);
	double expected_er = 1.25912877905715;
//	printf("||ERROR||=%10.15g\n",er);
//	printf("diff %g\n",er-expected_er);
	Close(er, expected_er);
}


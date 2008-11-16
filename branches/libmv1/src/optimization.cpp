#include "mv.h"

namespace mv {

vec sphere_to_disk(const vec &P)
{
	double norm = norm_2(P);
	int n = P.size();

	double fudge, nx, sign;
	if (P[n-1] < 0.0)
		sign = -1;
	else
		sign = 1;

	nx = 2*acos(sign*P[n-1]/norm);
	if (nx != 0.) {
		fudge = sign*nx/norm/sin(nx/2);
	} else {
		fudge = 0.0;
	}
	
	vec R(n-1);
	for (int i=0; i<n-1; i++) 
		R[i] = fudge*P[i];

	double magR = norm_2(R);
	assert(magR < M_PI + 1e-10);

	return R;
}

vec disk_to_sphere(const vec &R)
{
	int n = R.size() + 1;
	int i;
	double mag = norm_2(R);
	double fudge;
	vec P(n);
	if (mag != 0.) {
		fudge = sin(mag/2.0)/mag;
		for (i=0; i<n-1; i++)
			P[i] = fudge*R[i];
		P[n-1] = cos(mag/2.0); // now ||X|| = 1.0
	} else {
		for (i=0; i<n-1; i++)
			P[i] = 0.0;
		P[n-1] = 1.0;
	}

	return P;
}


mat jacobian_sphere_disk(const vec &X)
{
	int n = X.size();
	double nx = norm_2(X);
	double nx2 = nx/2.0;

	mat J(n+1,n);
	submat top(J, range(0,n), range(0,n));

	top = .25/nx2*(cos(nx2)-sin(nx2)/nx2)/nx* outer_prod(X,X);
	top += .5*sin(nx2)/nx2 * eye(n);
	
	for (int i=0; i<n; i++)
		J(n,i) = -X[i]/2./nx * sin(nx/2);
	return J;
}

} // namespace mv

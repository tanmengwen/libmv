#include "testmv.h"

TEST(two_by_two_invert) {
	double x[] = { 0.07342297,  0.96918938, 0.42715183,  0.73381101};
	mat X(2,2);
	fill(X,x);
	double inv_x[] = {-2.03772751,  2.69135216, 1.18616241, -0.20388902};
	mat invX(2,2);
	fill(invX,inv_x);
	inv2inplace(&X);
	mat residual = invX - X;
	AllSmall(residual);
}

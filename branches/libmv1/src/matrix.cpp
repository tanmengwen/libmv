
#include "mv.h"

namespace mv {


double norm(mat const &m) {
	double ret = 0.;
	for (size_t i=0; i<m.size1(); i++) {
		for (size_t j=0; j<m.size2(); j++) {
			ret += m(i,j)*m(i,j);
		}
	}
	return sqrt(ret);
}

void ptst(const vec &a, char* name) {
	printf("double %s_[] = {",name);
	foreach (double v, a) 
		printf("%g,", v);
	printf("};\n");
	printf("vec %s(%d); fill(%s,%s_);\n",name,a.size(),name,name);
}
void ptst(const mat &M, char* name) {
	printf("double %s_[] = {",name);
	for (size_t i=0; i<M.size1(); i++)
		for (size_t j=0; j<M.size2(); j++)
			printf("%g,", M(i,j));
	printf("};\n");
	printf("mat %s(%d,%d); fill(%s,%s_);\n",name,M.size1(),M.size2(),name,name);
}

void pnumpy(const vec &a) {
	printf("array([");
	foreach (double v, a) 
		printf("%.17g,", v);
	printf("])\n");
}

void pnumpy(const mat &M) {
	printf("array([\n");
	for (size_t i=0; i<M.size1(); i++) {
		printf("    [");
		for (size_t j=0; j<M.size2(); j++)
			printf("%g,", M(i,j));
		printf("],\n");
	}
	printf("]) # shape(%d,%d)\n", M.size1(), M.size2());
}

vec operator*(const vec& a, const vec& b)
{
	assert(a.size() == b.size());
	return element_prod(a,b);
}

vec operator-(const vec& a, double b)
{
	vec ret(a);
	for (size_t i=0; i<a.size(); i++)
		ret[i] -= b;
	return ret;
}

double amax(const vec& a)
{
	double maximum = -HUGE_VAL;
	foreach (double v, a) {
		if (v > maximum)
			maximum = v;
	}
	return maximum;
}

double amin(const vec& a)
{
	double minimum = HUGE_VAL;
	foreach (double v, a) {
		if (v < minimum)
			minimum = v;
	}
	return minimum;
}

vec aexp(const vec& a)
{
	vec ret(a);
	for (size_t i=0; i<a.size(); i++)
		ret[i] = exp(a[i]);
	return ret;
}

vec ravel(const mat &M)
{
	vec r(M.size1()*M.size2());
	for (size_t i=0; i<M.size1(); i++)
		for (size_t j=0; j<M.size2(); j++)
			r[i*M.size2()+j] = M(i,j);
	return r;
}

mat reshape(const vec &v, size_t rows, size_t cols)
{
	assert(v.size() == rows*cols);
	mat ret(rows,cols);

	for (size_t i=0; i<rows; i++)
		for (size_t j=0; j<cols; j++)
			ret(i,j) = v[cols*i+j];

	return ret;
}


void fill(vec &v, double val)
{
	for (size_t i=0; i<v.size(); i++)
		v[i] = val;
}

void fill(vec &v, double *val)
{
	int k=0;
	for (size_t i=0; i<v.size(); i++)
		v[i] = val[k++];
}

void fill(mat &M, double *val)
{
	int k=0;
	for (size_t i=0; i<M.size1(); i++)
		for (size_t j=0; j<M.size2(); j++)
			M(i,j) = val[k++];
}

void fill(mat &M, double val)
{
	for (size_t i=0; i<M.size1(); i++)
		for (size_t j=0; j<M.size2(); j++)
			M(i,j) = val;
}

void fill(fmat &M, double val)
{
	for (size_t i=0; i<M.size1(); i++)
		for (size_t j=0; j<M.size2(); j++)
			M(i,j) = val;
}

void zero(vec &v)
{
	fill(v, 0.);
}

void zero(mat &m)
{
	fill(m, 0.);
}

vec sum2(const mat &m)
{
	vec v(m.size1());
	for (size_t i=0; i<m.size1(); i++) {
		v[i] = 0.0; 
		for (size_t j=0; j<m.size2(); j++) 
			v[i] += m(i,j);
	}
	return v;
}

// Numeric style broadcasting
mat operator*(const mat& M, const vec& b)
{
	assert(M.size2() == b.size());
	mat out = M;

	for (size_t j=0; j<M.size1(); j++) {
		for (size_t k=0; k<M.size2(); k++) {
			out(j, k) = M(j,k) * b[k];
		}
	}
	return out;
}

// Numeric style broadcasting
mat operator*(const vec& b, const mat& M)
{
	assert(M.size1() == b.size());
	mat out = M;

	for (size_t j=0; j<M.size2(); j++) {
		for (size_t k=0; k<M.size1(); k++) {
			out(k,j) = M(k,j) * b[k];
		}
	}
	return out;
}

mat transpose(const mat &M)
{
	size_t n = M.size1();
	size_t m = M.size2();

	mat Mt(M.size2(), M.size1());
	for (size_t i=0; i<n; i++) 
		for (size_t j=0; j<m; j++) 
			Mt(j,i) = M(i,j);
	return Mt;
}

void inv2inplace(mat *mm) {
	mat &M = *mm;
	assert(M.size1() == 2);
	assert(M.size2() == 2);
	double a=M(0,0);
	double b=M(0,1);
	double c=M(1,0);
	double d=M(1,1);
	double det = a*d - b*c;
	M(0,0) = d;
	M(1,1) = a;
	M(0,1) = -b;
	M(1,0) = -c;
	M /= det;
}

/*
vec euler_axis_angle_from_rotation_matrix(mat R)
{
	vec e(3);
	double theta = acos((R(0,0) + R(1,1) + R(2,2) - 1)/2.);
	e[0] = R(2,1)-R(1,2);
	e[1] = R(0,2)-R(2,0);
	e[2] = R(1,0)-R(0,1);
	assert(theta > 0.00001);
	e /= 2*sin(theta);
	return e;
}

mat rotation_matrix_from_euler_axis_angle(vec e)
{
	double theta = norm(e);
	mat ecross = crossmat(e);
	mat I(eye(3));
	mat R = s

}
*/

void rq(const mat &A, mat &R, mat &Q)
{
	// Clear out bottom middle A21
	// need c*a21+s*a22=0
	// and -s*a21+c*a22 > 0
	mat Qx(3,3);
	if (A(2,1)!=0.0) {
		double c=A(2,2), s=-A(2,1);
		double d = sqrt(c*c + s*s);
		c /= d;
		s /= d;
		double _Qx[] = {1, 0, 0, 0, c,-s, 0, s, c};
		fill(Qx, _Qx);
	} else {
		Qx = eye(3);
	}
	mat AQx = dot(A,Qx);

	// Clear out bottom left AQx20
	// need c*a20-s*a22=0
	mat Qy(3,3);
	if (AQx(2,0) != 0.0) {
		double c=AQx(2,2), s=AQx(2,0);
		double d = sqrt(c*c + s*s);
		c /= d;
		s /= d;
		double _Qy[] = {c, 0, s, 0, 1, 0, -s, 0, c};
		fill(Qy, _Qy);
	} else {
		Qy = eye(3);
	}
	mat AQxQy = dot(AQx,Qy);

	// Clear out middle left AQxQy10
	// need c*a10+s*a11=0
	// also, if we want the diagonal to be positive, we may have to switch the
	// determinant of one of the rotations if the A matrix is orientation
	// reversing.
	mat Qz(3,3);
	if (AQxQy(1,0) != 0.0) {
		// FIXME assuming positive det... no det function!
		double c=AQxQy(1,1), s=-AQxQy(1,0);
		double d = sqrt(c*c + s*s);
		c /= d;
		s /= d;
		double _Qz[] = {c,-s, 0, s, c, 0, 0, 0, 1};
		fill(Qz, _Qz);
	} else {
		Qz = eye(3);
	}
	mat AQxQyQz = dot(AQxQy,Qz);
	R = AQxQyQz;
	mat QxT = trans(Qx);
	mat QyT = trans(Qy);
	mat QzT = trans(Qz);
	mat tmp = dot(QyT, QxT);
	Q = dot(QzT, tmp);
}

double det3(const mat &a)
{
	assert(a.size1() ==3);
	assert(a.size2() ==3);

	double det = 
		a(0,0) * (a(1,1)*a(2,2) - a(2,1)*a(1,2)) +
		a(0,1) * (a(1,2)*a(2,0) - a(2,2)*a(1,0)) +
		a(0,2) * (a(1,0)*a(2,1) - a(2,0)*a(1,1));

	return det;
}


void sqrtm(mat &M, mat *res) {
	assert(M.size1()==M.size2());
	bmat s;
	fmat U, Vt;
	s = svd(M, U, Vt);
	for (size_t i=0; i<M.size1(); i++)
		s(i,i) = sqrt(s(i,i));
	*res = dot(U, s);
	*res = dot(*res, Vt);
}


} // namespace mv

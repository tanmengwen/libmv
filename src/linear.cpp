#include "mv.h"
#include "linalg.h"


namespace mv {

//double dot(const vec &a, const vec &b)
//{
//	return inner_prod(a,b);
//}

mat crossmat(vec &v)
{
	double x = v[0];
	double y = v[1];
	double z = v[2];

	mat M(3,3);

	M(0,0) =  0.; M(1,0) =  z ; M(2,0) = -y ; 
	M(0,1) = -z ; M(1,1) =  0.; M(2,1) =  x ; 
	M(0,2) =  y ; M(1,2) = -x ; M(2,2) =  0.; 

	return M;
}

double nullspace(const fmat &M, vec &v)
{
	size_t n = M.size1();
	size_t m = M.size2();

	ublas::banded_matrix<double> s;
	fmat U(n,n);
	fmat Vt(m,m);

	s = mimas::gesvd<double>(M, &U, &Vt);

	v.resize(m,false);
	for (size_t i=0; i<m; i++)
		v[i] = Vt(m-1,i);

	if (n < m)
		return 0.0; // row deficient
	return s(m-1, m-1);
}

double nullspace(const mat &M, vec &v) 
{
	fmat Mt(M); // likely inefficient
	return nullspace(Mt,v);
}

double nullspace(const fmat &M, vec &v1, vec &v2)
{
	size_t n = M.size1();
	size_t m = M.size2();

	ublas::banded_matrix<double> s;
	fmat U(n,n);
	fmat Vt(m,m);

	s = mimas::gesvd<double>(M, &U, &Vt);

	v1.resize(m,false);
	v2.resize(m,false);
	for (size_t i=0; i<m; i++) {
		v1[i] = Vt(m-1,i);
		v2[i] = Vt(m-2,i);
	}

	if (n < m)
		return 0.0; 
	return s(m-1, m-1);
}

double nullspace(const mat &M, vec &v1, vec &v2) 
{
	fmat Mt(M); // likely inefficient
	return nullspace(Mt,v1,v2);
}

bmat svd(mat &M, fmat &U, fmat &Vt)
{
	fmat Mt(M);
	return svd(Mt, U, Vt);
}

bmat svd(fmat &M, fmat &U, fmat &Vt)
{
	return mimas::gesvd<double>(M, &U, &Vt);
}

mat inv(const mat &M)
{
	return mat(mimas::inv(fmat(M)));
}

mat inv3x3(mat &M)
{
	assert(M.size1()==3);
	assert(M.size2()==3);

	mat m(3,3);

	assert(0); // FIXME
}

mat pinv(const mat &M)
{
	int m = M.size1();
	int n = M.size2();

	ublas::banded_matrix<double> s;
	fmat U(m,m);
	fmat Vt(n,n);

	s = mimas::gesvd<double>(M, &U, &Vt);

	mat Winv(n,m);
	int i,j;
	for (i=0; i<m; i++) {
		for (j=0; j<n; j++) {
			if (i==j)
				Winv(i,i) = 1./s(i,i);
			else
				Winv(j,i) = 0.;
		}
	}

	mat V = transpose(Vt);
	mat Ut = transpose(U);
	Winv = dot(V,Winv);
	Winv = dot(Winv,Ut);

	return Winv;
}



} // namespace mv

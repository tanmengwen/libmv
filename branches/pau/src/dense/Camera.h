#ifndef __Camera_h__
#define __Camera_h__

#include <assert.h>

#include <vnl/vnl_double_3x4.h>
#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_4.h>
#include <vnl/vnl_inverse.h>


class Camera {
public:
	vnl_double_3x3 K;
	vnl_double_3x3 R;
	vnl_double_3   t;

	vnl_double_3x4 P;
	vnl_double_3x3 KR1;  // KR^{-1} for computing unprojections faster
	vnl_double_3 ocenter;

	int width, height;
	double near, far;


	/////////////////////////////////////////////////
	// Settings

	void set_P(const vnl_double_3x4 &P_);
	void set_KRt(const vnl_double_3x3 &K_, const vnl_double_3x3 &R_, const vnl_double_3 &t_);
private:
	bool check_K(const double tol=1e-8);
	void update_KR1();
	void update_optical_center();

public:
	vnl_double_3x4 Rt();
	void image_resize( int neww, int newh );
	void include_between_near_and_far(const double x, const double y, const double z);


	/////////////////////////////////////////////////
	// Projections

	void project( double &u,double &v,double &zi,
			const double x,const double y,const double z ) const {
		zi = P(2,0)*x + P(2,1)*y + P(2,2)*z + P(2,3);
		u = (P(0,0)*x + P(0,1)*y + P(0,2)*z + P(0,3))/zi;
		v = (P(1,0)*x + P(1,1)*y + P(1,2)*z + P(1,3))/zi;
	}


	// computes the projection of a 3D point and its gradient.
	// ux,...,vz are the derivatives of the projected coordinates u,v
	// with respect to the coordinates of the 3D point x,y,z
	void project_gradient( double &u,double &v,double &zi,
			double &ux,double &uy,double &uz,
			double &vx,double &vy,double &vz,
			const double x,const double y,const double z ) const
	{
		const double xx = P(0,0)*x + P(0,1)*y + P(0,2)*z + P(0,3);
		const double yy = P(1,0)*x + P(1,1)*y + P(1,2)*z + P(1,3);
		zi = P(2,0)*x + P(2,1)*y + P(2,2)*z + P(2,3);
		u = xx/zi;
		v = yy/zi;
		const double zi2=zi*zi;
		ux = (P(0,0)*zi - P(2,0)*xx)/zi2;
		uy = (P(0,1)*zi - P(2,1)*xx)/zi2;
		uz = (P(0,2)*zi - P(2,2)*xx)/zi2;
		vx = (P(1,0)*zi - P(2,0)*yy)/zi2;
		vy = (P(1,1)*zi - P(2,1)*yy)/zi2;
		vz = (P(1,2)*zi - P(2,2)*yy)/zi2;
	}

	void camcoords( double &xi,double &yi,double &zi,
			const double x,const double y,const double z ) const {
		xi = R(0,0)*x + R(0,1)*y + R(0,2)*z + t(0);
		yi = R(1,0)*x + R(1,1)*y + R(1,2)*z + t(1);
		zi = R(2,0)*x + R(2,1)*y + R(2,2)*z + t(2);
	}

	void image_camcoords(double &u,double &v,double &xi,double &yi,double &zi,
				const double x,const double y,const double z) const {
		camcoords(xi,yi,zi,x,y,z);
		const double xx=xi/zi;
		const double yy=yi/zi;
		u = K(0,0)*xx + K(0,1)*yy + K(0,2);
		v =             K(1,1)*yy + K(1,2);
	}

	/////////////////////////////////////////////////
	// UnProjections


	vnl_double_3 unproject(	const double u,const double v,const double zi ) const {
		vnl_double_3 U(u*zi,v*zi,zi);
		vnl_double_3 dir = KR1 * U;
		return ocenter + dir;
	}

	vnl_double_3 pixel_direction( const double x,const double y ) const {
		vnl_double_3 v(x,y,1);
		return KR1 * v;
	}

	vnl_double_3 intrinsic_pixel_direction(const double x,const double y) const {
		vnl_double_3 v(x,y,1);
		return vnl_inverse(K) * v;
	}


	/////////////////////////////////////////////////
	// OpenGL conversions

	void openGLprojection( double M[4][4] ) const;
	double zi_from_openGL_depth_buffer( const double d ) {
		// coeficients to compute the normalized z, see glFrustum doc
		const double C=(far+near)/(far-near);
		const double D=-2*far*near/(far-near);

		return D/(2*d-1 - C);
	}

	void openGLmodelview( double M[4][4] ) const;
	vnl_double_3x3 H_tex_coords_from_pixel_coords();
public:

	// find the P = K(R|t) decomposition HZ 6.1 pag.157
	static void KRt_from_P( vnl_double_3x3 &K, vnl_double_3x3 &R, vnl_double_3 &t, const vnl_double_3x4 &P );
	static vnl_double_3x3 tensor_product( const vnl_double_3 &a, const vnl_double_3 &b );
	static vnl_double_3x3 H_from_2P_plane( const Camera &c1, const Camera &c2,
					const vnl_double_4 &plane );
	// homogenuous coordinates of a fronto-parallel plane with depth d
	vnl_double_4 frontoparallel_plane_from_depth( double d );
};



class FrustumEstimator
{
public:
	Camera &c;
	double near,far;

	FrustumEstimator(Camera &cam) : c(cam)
	{
		near = 1e20;
		far = 0;
	}

	void push(const double x, const double y, const double z)
	{
		double u,v,zi;
		c.project(u,v,zi, x,y,z);
		if(zi<near) near=zi;
		if(far<zi) far=zi;
	}
};




#endif //__Camera_h__

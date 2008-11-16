#include "radial.h"
#include "json.h"
#include <mv.h>

extern "C" {
#include "lm.h"
}

namespace mv {

using json::Json;

Intrinsics::Intrinsics(const Json &params) {
	fx = params["fx"].as_double();
	fy = params["fy"].as_double();
	cx = params["cx"].as_double();
	cy = params["cy"].as_double();
	if (params.has_key("k1")) 
		k1 = params["k1"].as_double();
	if (params.has_key("k2")) 
		k2 = params["k2"].as_double();
	if (params.has_key("p1")) 
		p1 = params["p1"].as_double();
	if (params.has_key("p2")) 
		p2 = params["p2"].as_double();
}

void Intrinsics::init() {
	fx = 1.0;
	fy = 1.0;
	cx = 0.0;
	cy = 0.0;
	k1 = 0.0;
	k2 = 0.0;
	p1 = 0.0;
	p2 = 0.0;
}

void Intrinsics::distort_normalized(
		double x_undistorted,    double y_undistorted,
		double *x_distorted, double *y_distorted) const
{
	double xp = x_undistorted;
	double yp = y_undistorted;
	double r2 = xp*xp + yp*yp;
	double r4 = r2*r2;
	*x_distorted = xp*(1+k1*r2+k2*r4) + 2*p1*xp*yp + p2*(r2+2*xp*xp);
	*y_distorted = yp*(1+k1*r2+k2*r4) + 2*p2*xp*yp + p1*(r2+2*yp*yp);
}

void Intrinsics::distort(
		double x_undistorted, double y_undistorted,
		double *x_distorted,  double *y_distorted) const
{
	double xp = x_undistorted;
	double yp = y_undistorted;
	double u = (xp - cx) / fx;
	double v = (yp - cy) / fy;

	double ud, vd;
	distort_normalized(u, v, &ud, &vd);

	*x_distorted = fx*ud + cx;
	*y_distorted = fy*vd + cy;
}

static void forward_distort(double *p, double *hx, int m, int n, void *adata) {
	(void) n;
	(void) m;
	const Intrinsics *I = static_cast<const Intrinsics*>(adata);
	I->distort_normalized(p[0], p[1], hx+0, hx+1);
}

void Intrinsics::undistort(
		double x_distorted,    double y_distorted,
		double *x_undistorted, double *y_undistorted) const
{
	// undo K
	double uv[] = {(x_distorted - cx) / fx,
	               (y_distorted - cy) / fy};

	// do minimization in normalized coords
	double uvp[] = {uv[0], uv[1]};
	dlevmar_dif(forward_distort, uvp, uv, 2,2, 100/*maxits*/,
			NULL,NULL,NULL,NULL, const_cast<Intrinsics*>(this));
	
	// re-apply k
	*x_undistorted = fx*uvp[0] + cx;
	*y_undistorted = fy*uvp[1] + cy;
}

void undistort_track(const Intrinsics &k, TrackedSequence *ts) {

	for (size_t i=0; i<ts->size(); i++) {
		for (size_t j=0; j<ts->at(i)->size(); j++) {
			PointFeature *pf = static_cast<PointFeature*>(ts->at(i)->at(j));
			double xd = pf->x();
			double yd = pf->y();
			double xu, yu;
			k.undistort(xd,yd, &xu, &yu);
			pf->set(xu, yu);
		}
	}
}

} // namespace mv

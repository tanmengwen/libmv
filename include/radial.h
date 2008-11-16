#ifndef _RADIAL_H
#define _RADIAL_H

namespace json {
	class Json;
}

namespace mv {

class Intrinsics {
public:
	Intrinsics(const json::Json &js);
	void undistort(double x_distorted, double y_distorted,
	               double *x_undistorted, double *y_undistorted) const;
	void distort(double x_undistorted, double y_undistorted,
	             double *x_distorted, double *y_distorted) const;
	void distort_normalized(
	             double  u_undistorted, double v_undistorted,
	             double *u_distorted,  double *v_distorted) const;
	double fx, fy, cx, cy, k1, k2, p1, p2;
private:
	void init();
};

class TrackedSequence;
void undistort_track(const Intrinsics &k, TrackedSequence *ts);

} // namespace mv

#endif // _RADIAL_H

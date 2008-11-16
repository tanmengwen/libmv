#include <testmv.h>
#include "radial.h"

TEST(radial_io) {
	Json *intrinsics = json::Parse(
	"{ \"fx\" : 600.0, \"fy\" : 601.0, \"cx\" : 320.0, \"cy\" : 240.0,"
	" \"k1\" : -1.1e-4, \"k2\" : 0, \"p1\" : -2.1e-4, \"p2\" : 5 }");

	Intrinsics K(*intrinsics);

	Close(K.fx, 600.0);
	Close(K.fy, 601.0);
	Close(K.cx, 320.0);
	Close(K.cy, 240.0);
	Close(K.k1, -1.1e-4);
	Close(K.k2, 0.0);
	Close(K.p1, -2.1e-4);
	Close(K.p2, 5);
	delete intrinsics;
}

TEST(undistort_does_not_move_center) {
	Json *intrinsics = json::Parse(
	"{ \"fx\" : 600.0, \"fy\" : 601.0, \"cx\" : 320.0, \"cy\" : 240.0,"
	" \"k1\" : -1.1e-4, \"k2\" : 0, \"p1\" : -2.1e-4, \"p2\" : 5 }");
	Intrinsics K(*intrinsics);
	double x,y, xd,yd;
	x = 320;
	y = 240;
	K.distort(x,y,&xd,&yd);
	VeryClose(x, xd);
	VeryClose(y, yd);
	delete intrinsics;
}

TEST(undistort) {
	Json *intrinsics = json::Parse(
	"{ \"fx\" : 600.0, \"fy\" : 601.0, \"cx\" : 320.0, \"cy\" : 240.0,"
	" \"k1\" : -1.1e-4, \"k2\" : 0, \"p1\" : -2.1e-4, \"p2\" : 5 }");

	Intrinsics K(*intrinsics);

	double x,y, xd,yd, xp,yp;

	x = 330;
	y = 230;
	K.distort(x,y,&xd,&yd);
//	printf("xd=%g, yd=%g\n", xd,yd);
	NotClose(x, xd);
	NotClose(y, yd);
	K.undistort(xd,yd,&xp,&yp);
//	printf("xp=%g, yp=%g\n", xp,yp);
	CloseEps(x, xp, 0.01); /* 0.1 pixels is about the best we can hope for */
	CloseEps(y, yp, 0.01);
}

TEST(undistort) {
	Json *intrinsics = json::Parse(
	"{ \"fx\" : 669.9, \"fy\" : 671.1,   \"cx\" : 327.6, \"cy\" : 246.8,"
	"  \"k1\" : -0.1073258554724268,     \"k2\" : 0.0641739886773531,"
	"  \"p1\" : -2.8359702700842725e-04, \"p2\" : -6.3022225170024653e-04 }");

	Intrinsics K(*intrinsics);

	double x,y, xd,yd, xp,yp;

	x = 30;
	y = 20;
	x = 1.;
	y = 2.;
	K.distort(x,y,&xd,&yd);
//	printf("xd=%g, yd=%g\n", xd,yd);
	NotClose(x, xd);
	NotClose(y, yd);
	K.undistort(xd,yd,&xp,&yp);
//	printf("xp=%g, yp=%g\n", xp,yp);
	CloseEps(x, xp, 0.01); /* 0.1 pixels is about the best we can hope for */
	CloseEps(y, yp, 0.01);
}

static void is_distorted(const Intrinsics &k, double ox, double oy, PointFeature *pf) {
	double dx, dy;
	k.distort(pf->x(), pf->y(), &dx, &dy);
	Close(ox, dx);
	Close(oy, dy);
}

TEST(undistort_entire_track) {
	Json *intrinsics = json::Parse(
	"{ \"fx\" : 669.9, \"fy\" : 671.1,   \"cx\" : 327.6, \"cy\" : 246.8,"
	"  \"k1\" : -0.1073258554724268,     \"k2\" : 0.0641739886773531,"
	"  \"p1\" : -2.8359702700842725e-04, \"p2\" : -6.3022225170024653e-04 }");
	std::cout << intrinsics->to_string_pretty() << std::endl;
	Intrinsics K(*intrinsics);
	TrackedSequence ts;
	ts.add_point_feature(0,0, 1.0,2.0);
	ts.add_point_feature(1,0, 2.0,4.0);
//	std::cout << std::endl;
//	std::cout << ts.dump_json()->to_string_pretty() << std::endl;

	undistort_track(K, &ts); // under test

//	std::cout << std::endl;
//	std::cout << ts.dump_json()->to_string_pretty() << std::endl;

	is_distorted(K, 1.0, 2.0, static_cast<PointFeature*>(ts[0]->at(0)));
	is_distorted(K, 2.0, 4.0, static_cast<PointFeature*>(ts[1]->at(0)));
}

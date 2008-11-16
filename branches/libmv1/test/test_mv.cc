#include "testmv.h"

double points[] = {
           6.30119939,  861.79113542,    1.,
         944.38846589,  121.87784512,    1.,
         118.5687968 ,  247.5989237 ,    1.,
         147.6126068 ,  945.35357713,    1.,
         761.77204049,   66.67938243,    1.,
         268.70759373,  350.34912811,    1.,
         684.365766  ,  411.55107572,    1.,
         749.1629161 ,  363.61517314,    1.,
         836.12694931,  363.45339853,    1.,
         862.13731973,  195.37306295,    1.};

double h[] = {
         0.0035128 ,  0.        , -1.88958376 ,
         0.        ,  0.0035128 , -1.379701   ,
         0.        ,  0.        ,  1.         };

TEST() {
	mat mpoints(3,10);
	for (int i=0; i<3; i++) 
		for (int j=0; j<10; j++) 
			mpoints(i,j) = points[3*j+i];

	mat H = normalizing_homography(mpoints);

	for (int i=0; i<3; i++) 
		for (int j=0; j<3; j++) 
			Close(H(i,j), h[3*i+j]);
}

double testpts[] = {
  321.761404077,   242.10751656,              1.0,
  303.346994104,   231.710929302,             1.0,
  310.059683311,   238.217456879,             1.0,
   287.31955297,   216.549873354,             1.0,
  268.880061613,   212.281136469,             1.0,
  269.578114253,   230.562924283,             1.0,

  277.680261583,   232.533055574,             1.0,
  296.872804218,   256.899897232,             1.0,
  289.717190021,   229.854152807,             1.0,
  311.97928411,    238.597451718,             1.0,
  330.173754684,   224.534012824,             1.0,
  331.356015305,   218.749934678,             1.0,

  277.808086408,   249.573391256,             1.0,
  296.806907352,   214.836062197,             1.0,
   289.66702045,   248.364378201,             1.0,
  312.730811571,   217.947271485,             1.0,
  332.501391817,   227.709711752,             1.0,
   332.19035955,   251.829834601,             1.0,
};


TEST() {
	mat pts(18,3);
	fill(pts, testpts);

	std::vector<ThreeFrameTrack> tracks;

	for (int i=0; i<6; i++) {
		ThreeFrameTrack tft;
		for (int j=0; j<3; j++) {
			PointFeature *f = new PointFeature();
			f->assign(row(pts,6*j+i));
			tft[j] = f;
			Equals(tft[j], f);
		}
		tracks.push_back(tft);
	}

	std::vector<ProjectiveReconstruction> res;

	six_point_three_view(tracks, res);

//	printf("solutions: %d\n", res.size());
	for (size_t i=0; i<res.size(); i++) {
//		printf("reprojection error: %g\n", res[i].rms_reprojection_error());
		VerySmall(res[i].rms_reprojection_error());
//		res[i].bundle_adjust();
	}

	vec x1, x2;
	PointFeature x3;
	x1 = *(tracks[0])[0];
	x2 = *(tracks[0])[1];
	x3 = *(tracks[0])[2];
	x1[0] += 1.;
	x1[1] -= 1.;
	x2[0] -= 1.;
	x2[1] += 1.;
	mat F;
	ProjectiveCamera P1, P2;
	P1.assign(*(ProjectiveCamera*)(res[0].cameras[0]));
	P2.assign(*(ProjectiveCamera*)(res[0].cameras[1]));
	find_fundamental_from_camera_matricies(P1, P2, F);

	vec x1p(3), x2p(3);
	find_optimal_points_given_fundamental(x1, x2, F, x1p, x2p);

//	dpmat(x1);
//	dpmat(x2);
//	dpmat(x1p);
//	dpmat(x2p);
	PointStructure X;
	X.assign(triangulate_dlt(P1, P2, x1p, x2p));
	double err_optimal = ((ProjectiveCamera*)(res[0].cameras[2]))->reprojection_error(&X, &x3);

//	printf ("\nMY RESIDUALS: %g\n", sqrt(err));
//	dpmat(F);

	x1p.assign(x1);
	x2p.assign(x2);
	X.assign(triangulate_dlt(P1, P2, x1p, x2p));
	double err_dlt = ((ProjectiveCamera*)(res[0].cameras[2]))->reprojection_error(&X, &x3);

	// The optimal triangulation should beat the dlt!
	LessThan(err_optimal, err_dlt);

//	printf("(squared) error_dlt=%g, error_optimal=%g\n",err_dlt,err_optimal);
}

double basis[] = {
	1,0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1,
	1,1,1,1};

#undef dpmat
#define dpmat(x) 
void test_reprojection(mat& P, mat&x){

	mat Est(5,4);
	fill(Est, basis);
	mat Es = transpose(Est);
	mat tmp = dot(P, Es);

	dpmat(Es);
	for (int i=0; i<5; i++) {
		tmp(0,i) /= tmp(2,i);
		tmp(1,i) /= tmp(2,i);
		tmp(2,i) /= tmp(2,i);
	}
	dpmat(x);
	dpmat(tmp);
	tmp -= x;
	dpmat(tmp);
	AllVerySmall(tmp);
}

TEST() {
	mat pts(5,3);
	fill(pts, testpts);
	pts = transpose(pts);

	mat A,B;
	five_point_camera_pencil(pts, A,B);

	test_reprojection(A, pts);
	test_reprojection(B, pts);
	double mu = 0.1, nu = 10;
	mat P = mu*A + nu*B;
}

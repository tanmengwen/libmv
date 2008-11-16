#include "testmv.h"
#include "ransac.h"

#if 0
TEST() {

	char format[] = "/home/keir/thesis/movies/blender_test1/%04d.pgm";

	TrackedSequence ts;
	ts.enable_debug_output();
	ts.track_pgm_sequence(format, 31, 33, 500);

	Frame *frames[3];
	for (int i=0; i<3; i++)
		frames[i] = ts[i];
	ThreeViewSixPointRANSACDriver drv(frames);

	drv.search(100);
	printf("BEST:%g\n",drv.best_score);
	drv.best_fit.bundle_adjust();
}
#endif


#if 0

TEST() {
	char format[] = "/home/keir/thesis/movies/blender_test1/%04d.pgm";

	TrackedSequence ts;
	ts.enable_debug_output();
	ts.track_pgm_sequence(format, 31, 35, 500);

	Frame *frames[3];
	for (int i=0; i<3; i++)
		frames[i] = ts[i];
	ThreeViewSixPointRANSACDriver drv(frames);

	drv.search(100);
	printf("BEST:%g\n",drv.best_score);
	drv.best_fit.bundle_adjust();
}

#endif

#if 0
// merge code
TEST() {
	char format[] = "/home/keir/thesis/movies/blender_test1/%04d.pgm";

	TrackedSequence ts;
	ts.enable_debug_output();
	ts.track_pgm_sequence(format, 31, 35, 500);

	Frame *frames[3];
	for (int i=0; i<3; i++)
		frames[i] = ts[i];
	ThreeViewSixPointRANSACDriver drv(frames);

	drv.search(100);
	printf("BEST:%g\n",drv.best_score);
//	drv.best_fit.bundle_adjust();

	for (int i=0; i<3; i++)
		frames[i] = ts[i+2];
	ThreeViewSixPointRANSACDriver drv2(frames);

	drv2.search(100);
	printf("BEST:%g\n",drv2.best_score);
//	drv2.best_fit.bundle_adjust();

	mat H(4,4);

//	printf("RMS before norm: %g\n", drv.best_fit.rms_reprojection_error());
//	drv.best_fit.normalize_camera(0);
//	printf("RMS after norm: %g\n", drv.best_fit.rms_reprojection_error());

//	drv2.best_fit.normalize_camera(0);
//	drv.best_fit.align_oneview(&drv2.best_fit);
//	drv.best_fit.merge(&drv2.best_fit);

	printf("merge RMS: %g\n", drv.best_fit.rms_reprojection_error());
	drv.best_fit.bundle_adjust();
	printf("merge RMS bundled: %g\n", drv.best_fit.rms_reprojection_error());
}
#endif

TEST() {

	char track_fn[] = "test_data/blender1_37frame.ts";
	char reconstruction_fn[] = "test_data/blender1_37frame.pr";
	TrackedSequence *tss = read_from_file(track_fn);
	ProjectiveReconstruction *prr = read_reconstruction_from_file(*tss, reconstruction_fn);

	printf("# UPGRADE TO EUCLIDEAN\n");
	double kk[] = {875.5, 0., 400., 0,875.5, 300., 0.,0.,1.};
	mat K(3,3);
	fill(K,kk);
	find_euclidean_upgrade_calibrated(*prr, K);
	prr->print_cameras();
	printf("# STRUCTURE\n");
}

#if 0
#include "qfits.h"
TEST() {
	/* 4u frame 4u track 8f x 8f y = 24 bytes wide */
	qfits_header* hd = qfits_table_prim_header_default();
	qfits_header_add(hd, "COMMENT", "libmv point track",NULL,NULL);
	char buf[40];
	sprintf(buf, "%u", 10);
	qfits_header_add(hd, "NTRACKS", buf, "number of tracks",NULL);
	qfits_header_add(hd, "NFRAMES", buf, "number of frames",NULL);
	qfits_header_add(hd, "NPOINTS", buf, "number of points in all frames",NULL);

	int ncols=4;
	int nrows=10;
	qfits_table* table = qfits_table_new("", QFITS_BINTABLE, tablesize, ncols, nrows);
	qfits_col_fill(table->col, datasize, 0, 4, TFITS_BIN_TYPE_J,
		"qidx", "", "", "", 0, 0, 0, 0, 0);


	FILE* fid = fopen("test.fits", "wb");
	assert(fid);

}
#endif

TEST() {
//	char track_fn[] = "test_data/test_track_5_frames.ts";
//	char reconstruction_fn[] = "test_reconstruction_dump.pr";
//	char track_fn[] = "test_track_dump.ts";
//	char track_fn[] = "test_data/blender1_37frame.ts";
	char track_fn[] = "test_data/blender1_5frame.ts";
	TrackedSequence *tss = read_from_file(track_fn);

	ProjectiveReconstruction *pr = reconstruct_hierarchical(*tss,200, 5);
	pr->print_cameras();
}


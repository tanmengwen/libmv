#include "testmv.h"

// FIXME make the build aware of this...
#define HAVE_TEST_PGMS
#ifdef HAVE_TEST_PGMS

extern "C" {
#include "pnmio.h"
}

TEST() {
	TrackedSequence ts;
	ts.start_tracking(800, 600);
	Equals(ts.get_width(), 800);
	Equals(ts.get_height(), 600);
}

char format[] = "/home/keir/thesis/movies/blender_test1/%04d.pgm";
int real_startframe = 28;
int real_endframe = 64;

// Note that this is quite tied to the specific test images
TEST() {
	TrackedSequence ts;

	ts.enable_debug_output();

	unsigned char *img1, *img2;
	char fnamein[100];
	int startframe = 28;
//	int endframe = 64;
	int endframe = 30;
	int ncols, nrows;
	int i;
	int nfeats = 100;

	sprintf(fnamein, format, startframe);
	img1 = pgmReadFile(fnamein, NULL, &ncols, &nrows);
	img2 = (unsigned char *) malloc(ncols*nrows*sizeof(unsigned char));
	ts.start_tracking(ncols, nrows, nfeats);
	ts.push_frame(img1);
	Equals(ts[0]->size(), (size_t) nfeats);

	for (i=startframe+1; i<=endframe; i++)  {
		sprintf(fnamein, format, i);
		pgmReadFile(fnamein, img2, &ncols, &nrows);
		ts.push_frame(img2);

		if (i==startframe+1) 
			Equals(ts.get_ntracks(), (size_t) nfeats + 78);
		if (i==startframe+2) 
			Equals(ts.get_ntracks(), (size_t) nfeats + 78 + 78);
	}

	Equals(ts.size(), (size_t) endframe-startframe+1);

	ts.stop_tracking();
}

TEST() {
	TrackedSequence ts;
	ts.track_pgm_sequence(format, 28, 30);
	Equals(ts.size(), (size_t)3);
	size_t nfeats = ts[0]->size();
	size_t ntracks = ts.tracks.size();
//	printf("num tracks=%d\n",ts.tracks.size()); 
	ts.drop_track(ts.tracks[1]); // track 1 gets to the 2nd frame
	Equals(ts[0]->size(), nfeats-1);
	Equals(ts.tracks.size(), ntracks-1);
}

#endif // HAVE_TEST_PGMS

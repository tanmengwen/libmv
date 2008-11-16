#include <stdio.h>
#include "mv.h"
using namespace mv;
char* OPTIONS = "ht:p:abc";

void print_help_and_exit(char* progname) {
	printf("print out a track matrix for a reconstruction\n");
	printf("usage: %s -t <track_file.ts> [-p <reconstruction.pr>] [-abc]\n", progname);
	printf("          -a: print tracks independent of reconstruction\n");
	printf("          -b: tracks which made it to the reconstruction\n");
	printf("          -c: only the exact measurements kept in the reconstruction\n");
	exit(-1);
}

int main(int argc, char* argv[])
{
	char *track_fn=NULL;
	char *reconstruction_fn=NULL;
	int mode = 0;

	char argchar;
	while ((argchar = getopt (argc, argv, OPTIONS)) != -1) {
		switch (argchar) {
		case 'h': print_help_and_exit(argv[0]);   break;
		case 't': track_fn = optarg;              break;
		case 'p': reconstruction_fn = optarg;     break;
		case 'a': mode = 0;                       break;
		case 'b': mode = 1;                       break;
		case 'c': mode = 2;                       break;
		default:  print_help_and_exit(argv[0]);
		}
	}
	if (!track_fn)
		print_help_and_exit(argv[0]);

	if (!reconstruction_fn && mode > 0)
		print_help_and_exit(argv[0]);

	fprintf(stderr, "Loading tracked sequence %s...\n", track_fn);
//	TrackedSequence *ts = read_from_file(track_fn);
	TrackedSequence *ts = read_json_track(track_fn);
	if (ts == NULL) {
		fprintf(stderr, "error reading file...%s ?\n", track_fn);
		exit(-1);
	}

	ProjectiveReconstruction *prr = NULL;
	if (mode > 0) {
		fprintf(stderr, "Loading reconstruction...\n");
		prr = read_reconstruction_from_file(*ts, reconstruction_fn);
		if (prr == NULL) {
			fprintf(stderr, "error reading file... %s?\n", reconstruction_fn);
			exit(-1);
		} 
	} 

	/*
	 * print track matrix...
	 *
	*/

	int n = ts->tracks.size();
	if (mode > 0) 
		n = prr->structure.size();
	int m = ts->size();
	mat track_matrix(n, m);
	fill(track_matrix, 0.);

	if (mode == 0) {
		for (int i=0; i<n; i++) {
			Track* t = ts->tracks[i];
			int o = t->size();
			for (int j=0; j<o; j++) {
				int fr = (*t)[j]->get_frame_number();
				track_matrix(i,fr) = 1.0;
			}
		}
	} else if (mode == 1) {
		for (int i=0; i<n; i++) {
			Track* t = prr->structure[i]->get_track();
			for (size_t j=0; j<t->size(); j++) {
				size_t fr = (*t)[j]->get_frame_number();
				track_matrix(i,fr) = 1.0;
			}
		}
	} else {
		MeasurementVisitor mv2(prr);
		while (mv2.next()) {
			size_t ptn = mv2.structure()->get_number();
			size_t frn = mv2.camera()->get_frame_number();
			track_matrix(ptn, frn) = 1.;
		}
	}
	printf("from numpy import *\n");
	dpmat(track_matrix);
	printf("firsts = [argmax(v) for v in track_matrix]\n");
	printf("sf = argsort(firsts)\n");
	printf("track_matrix_fixed = track_matrix[sf,:]\n");
}

#include <stdio.h>
#include <vector>
#include "mv.h"
using namespace mv;
char* OPTIONS = "ht:p:m:e:s";

void print_help_and_exit(char* progname) {
	printf("usage: %s -t <track_file.ts> [-p <recon.pr>] [-m <multi.ppr>] [-e <euclid.mr]\n", progname);
	printf(" -h - this message\n");
	printf(" -p - projective reconstruction to print\n");
	printf(" -m - multi reconstruction to print\n");
	printf(" -e - euclidean reconstruction to print\n");
	printf(" -s - only print summary stats\n");
	exit(-1);
}

int main(int argc, char* argv[])
{
	char *track_fn = NULL;
	char *projective_fn = NULL;
	char *multi_fn = NULL;
	char *euclidean_fn = NULL;
	char *progname = argv[0];
	int summary = 0;

	char argchar;
	while ((argchar = getopt (argc, argv, OPTIONS)) != -1) {
		switch (argchar) {
		case 'h': print_help_and_exit(progname);   break;
		case 't': track_fn = optarg;               break;
		case 'p': projective_fn = optarg;          break;
		case 'm': multi_fn = optarg;               break;
		case 'e': euclidean_fn = optarg;           break;
		case 's': summary = 1;                     break;
		default:  print_help_and_exit(progname);
		}
	}

	if (!track_fn) {
		fprintf(stderr, "ERROR: must supply a track file.\n");
		print_help_and_exit(progname);
	}

	TrackedSequence *ts = read_from_file(track_fn);

	if (ts == NULL) {
		fprintf(stderr, "ERROR: couldn't read %s\n", track_fn);
		exit(-1);
	}

	if (projective_fn) {
		ProjectiveReconstruction* pr = read_reconstruction_from_file(
				*ts, projective_fn);
		if (pr == NULL) {
			fprintf(stderr, "ERROR: couldn't read %s\n", projective_fn);
			exit(-1);
		}
		printf("# PROJECTIVE RECONSTRUCTION\n");
		printf("# rms  error=%g\n",pr->rms_reprojection_error());
		printf("# mean error=%g\n",pr->mean_reprojection_error());
		printf("from numpy import *\n");
		printf("# CAMERAS\n");
		pr->print_cameras();
		printf("# STRUCTURE\n");
		pr->print_structure();
		printf("# RESIDUALS\n");
		pr->print_residuals();
	}

	if (multi_fn) {
//		ProjectiveReconstruction *prr;
//		std::vector<ProjectiveReconstruction*> prs;
//		read_many_reconstructions(*ts, multi_fn, &prs);
		fprintf(stderr, "ERROR: unimplemented.\n");
		assert(0);
	}

	if (euclidean_fn) {
		fprintf(stderr, "Loading reconstruction %s...\n", euclidean_fn);
		EuclideanReconstruction *mr = read_euclidean_reconstruction_from_file(*ts, euclidean_fn);
		if (mr == NULL) {
			fprintf(stderr, "ERROR: couldn't open reconstruction file %s\n", euclidean_fn);
			exit(-1);
		}
		if (summary != 1) {
			printf("# PROJECTIVE RECONSTRUCTION\n");
			printf("# rms  error=%g\n",mr->rms_reprojection_error());
			printf("# mean error=%g\n",mr->mean_reprojection_error());
			printf("from numpy import *\n");
			printf("# CAMERAS\n");
			mr->print_cameras();
			printf("# STRUCTURE\n");
			mr->print_structure();
			printf("# RESIDUALS\n");
			mr->print_residuals();
		} 

		printf("# number of cameras: %d\n", mr->cameras.size());
		printf("# number of points: %d\n", mr->structure.size());
		printf("# frames from tracked sequence: ");
		for (size_t i=0; i<mr->cameras.size(); i++) {
			printf("%d=>%d, ", i, mr->cameras[i]->get_frame_number());
		}
		printf("\n");
	}
}

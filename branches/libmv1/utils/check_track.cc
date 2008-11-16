#include <stdio.h>
#include "mv.h"
#include "ground_truth.h"
using namespace mv;
char* OPTIONS = "ht:g:o:";

void print_help_and_exit(char* progname) {
	printf("generate a blender export of the ground truth\n");
	printf("usage: %s -t <track_file.ts> -o <export.py>\n", progname);
	printf("  -h           this message\n");
	printf("  -t<track>    track data\n");
	printf("  -g<truth>    ground truth to evaluate against\n");
	printf("  -o<int>      output filename\n");
	exit(-1);
}

void exit_error(char* progname, char *msg) {
	fprintf(stderr, msg);
	print_help_and_exit(progname);
}
#define ERROR(msg) exit_error(argv[0], "ERROR: " msg "\n")

int main(int argc, char* argv[])
{
	char *track_fn=NULL;
	char *ground_truth_fn=NULL;
	char *output_fn=NULL;
	int first=0, last=0;

	char argchar;
	while ((argchar = getopt (argc, argv, OPTIONS)) != -1) {
		switch (argchar) {
		case 'h': print_help_and_exit(argv[0]);   break;
		case 't': track_fn = optarg;              break;
		case 'g': ground_truth_fn = optarg;       break;
		case 'o': output_fn = optarg;             break;
		default:  print_help_and_exit(argv[0]);
		}
	}

	if (!track_fn)
		ERROR("missing track data");

	if (!ground_truth_fn)
		ERROR("need exr ground truth data");

	if (!output_fn)
		ERROR("no output filename");

	fprintf(stderr, "Loading tracked sequence %s...\n", track_fn);
	TrackedSequence *ts = read_from_file(track_fn);
	if (ts == NULL) {
		fprintf(stderr, "ERROR: couldn't read file %s\n", track_fn);
		exit(-1);
	}

	GroundTruthEXRLoader loader;
	loader.load_ground_truth(ground_truth_fn, first, last);
	GroundTruth truth;
	truth.set_loader(&loader);

	// push structure and measurements into ground truth (and force loads)
	// Structure
	for (size_t i=0; i<ts->size(); i++) {
		for (size_t j=0; j<ts->at(i)->size(); j++) {
		PointFeature *pf = (PointFeature*) ts->at(i)->at(j);
			truth.add_point(1+ts->at(i)->at(j)->get_frame_number(), 0,
			                pf->x(), pf->y());
		}
	}

	truth.export_truth_blender(output_fn);
	printf("Exported to %s\n", output_fn);
}

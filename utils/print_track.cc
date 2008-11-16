#include <stdio.h>
#include "mv.h"
using namespace mv;
char* OPTIONS = "ht:";

void print_help_and_exit(char* progname) {
	printf("print information about a tracked sequence\n");
	printf("usage: %s -t <track_file.ts> [-p <reconstruction.pr>] [-abc]\n", progname);
	exit(-1);
}

int main(int argc, char* argv[])
{
	char *track_fn=NULL;
	char argchar;
	while ((argchar = getopt (argc, argv, OPTIONS)) != -1) {
		switch (argchar) {
		case 'h': print_help_and_exit(argv[0]);   break;
		case 't': track_fn = optarg;              break;
		default:  print_help_and_exit(argv[0]);
		}
	}
	if (!track_fn)
		print_help_and_exit(argv[0]);

	fprintf(stderr, "Loading tracked sequence %s...\n", track_fn);
	TrackedSequence *ts = read_from_file(track_fn);
	if (ts == NULL) {
		fprintf(stderr, "error reading file...%s ?\n", track_fn);
		exit(-1);
	}

	printf("frame      features     filename\n"); 
	for (size_t i=0; i<ts->size(); i++) {
		printf("%-10d %-10d %10s\n",
				ts->at(i)->get_frame_number(),
				ts->at(i)->size(),
				ts->at(i)->get_filename().c_str());
	}
}


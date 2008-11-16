#include <stdio.h>
#include "mv.h"
using namespace mv;
char* OPTIONS = "ht:";

void print_help_and_exit(char* progname) {
	printf("create a .c file out of a tracked sequence\n");
	printf("usage: %s -t <track_file.ts>\n", progname);
	printf("  -h           this message\n");
	printf("  -t<track>    track data\n");
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

	char argchar;
	while ((argchar = getopt (argc, argv, OPTIONS)) != -1) {
		switch (argchar) {
		case 'h': print_help_and_exit(argv[0]);   break;
		case 't': track_fn = optarg;              break;
		default:  print_help_and_exit(argv[0]);
		}
	}

	if (!track_fn)
		ERROR("missing track data");

	fprintf(stderr, "Loading tracked sequence %s...\n", track_fn);
	TrackedSequence *ts = read_from_file(track_fn);
	if (ts == NULL) {
		fprintf(stderr, "ERROR: couldn't read file %s\n", track_fn);
		exit(-1);
	}

	string dump = ts->dump_cpp_ascii();
	printf("%s", dump.c_str());
}

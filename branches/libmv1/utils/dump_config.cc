#include <stdio.h>
#include <json.h>
#include "config.h"
#include "extern/google/gflags.h"

using namespace mv;

DEFINE_string(field_to_print, "config", "Which field from the JSON file to print.");

void exit_error(char *msg) {
	fprintf(stderr, msg);
	exit(-1);
}
#define ERROR(msg) exit_error("ERROR: " msg "\n")

int main(int argc, char* argv[])
{
	string usage(
			"<json file>: dump the \"config\" field from a json file\n");
	google::SetUsageMessage(usage);
	google::ParseCommandLineFlags(&argc, &argv, true);

	if (argc != 2)
		ERROR("No file specified");

	json::Json *js = json::FromFile(argv[1]);
	if (js->has_key(FLAGS_field_to_print)) {
		json::Json &config = js->get(FLAGS_field_to_print);
		printf("%s\n", config.to_string_pretty().c_str());
	} else {
		fprintf(stderr, "ERROR: File %s has no field %s.\n",
				argv[0], FLAGS_field_to_print.c_str());
		return -1;
	}
}

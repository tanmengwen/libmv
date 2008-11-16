#include <testmv.h>
#include "config.h"

TEST(parameter_retrevial_and_save) {
	Config config;
	double foobar = 1.0;
	config.parameter("foobar", 0.96, &foobar);
	Equals(0.96, foobar);
}

TEST(parameter_retrevial_and_save_boolean) {
	Config config;
	bool toggle = false;
	config.parameter("foobar", true, &toggle);
	Equals(true, toggle);
}

TEST(has_parameter) {
	Config config;
	double foobar;
	Equals(false, config.has_parameter("foobar"));
	config.parameter("foobar", 0.96, &foobar);
	Equals(true, config.has_parameter("foobar"));
}

TEST(can_construct_hierarchical_config) {
	Config config;
	Equals(false, config.has_parameter("subconfig"));
	Config subconfig = config.subconfig("subconfig");
	double knob = 1.0;
	subconfig.parameter("knob", 12.3, &knob);
	Equals(true, subconfig.has_parameter("knob"));
	Equals(true, config.has_parameter("subconfig"));
}


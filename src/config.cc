#include "config.h"
#include "extern/google/gflags.h"

DEFINE_string(config, "", "Libmv config file to load defaults from");

namespace mv {

Config LoadConfig() {
	if (FLAGS_config.size()) {
		json::Json *js = json::FromFile(FLAGS_config.c_str());
		Config con(js);
		con.set_view();
		return con;
	}
	Config con;
	con.set_view();
	return con;
}

// FIXME add stuff to bake in bzr revision

} // namespace mv

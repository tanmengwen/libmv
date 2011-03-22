#ifdef __APPLE__
#include "gflags_config_mac.h"
#elif __GNUC__
#include "gflags_config_linux.h"
#elif _MSC_VER
#include "windows/config.h"
#endif


#ifndef _KEYFRAME_H
#define _KEYFRAME_H
#include "mv.h"
namespace mv {
void pick_keyframes(const TrackedSequence &ts, std::vector<size_t> *keyframes,
		Config *config);
}
#endif // _KEYFRAME_H

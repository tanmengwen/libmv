#ifndef _STRUCTURE_H
#define _STRUCTURE_H

#include "mv.h"

namespace mv {

PointStructure* json_to_point_structure(TrackedSequence &ts, const Json &js);
EuclideanPointStructure* json_to_euclidean_point_structure(TrackedSequence &ts, const Json &js);

} // namespace mv

#endif // _STRUCTURE_H

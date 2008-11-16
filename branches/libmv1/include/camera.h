#ifndef _CAMERA_H
#define _CAMERA_H

#include "mv.h"

namespace mv {

EuclideanCamera* json_to_euclidean_camera(TrackedSequence &ts, Json* js);
ProjectiveCamera* json_to_projective_camera(TrackedSequence &ts, Json* js);
Camera *json_to_camera(TrackedSequence &ts, Json& js);

}

#endif // _CAMERA_H

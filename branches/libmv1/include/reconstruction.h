#ifndef _RECONSTRUCTION_H
#define _RECONSTRUCTION_H

#include "mv.h"

namespace mv {

Reconstruction *json_to_reconstruction(TrackedSequence &ts, const Json &js);
void json_to_reconstructions(TrackedSequence &ts, const Json &js,
		std::vector<Reconstruction*> *prs);
Json* reconstructions_to_json(
		const std::vector<Reconstruction*> &prs);

} // namespace mv

#endif // _RECONSTRUCTION_H

#ifndef _ALIGN_TRUTH_H
#define _ALIGN_TRUTH_H
#include "mv.h"

namespace mv {

void find_aligning_transform(const mat& A, const mat& B, mat *T);
bool robust_find_aligning_transform(const mat& A, const mat& B, mat *T, double *error_thresh);
void find_aligning_projective_transform(const mat& A, const mat& B, mat *T);

} // namespace mv

#endif // _GROUND_TRUTH_H

#ifndef _ROBUST_SIXPOINTNVIEW_H
#define _ROBUST_SIXPOINTNVIEW_H

#include "mv.h" 

namespace mv {

ProjectiveReconstruction *robust_sixpoint_n_view(
		mat &points, double *error_thresh);

ProjectiveReconstruction *robust_sixpoint_n_view_tracks(
		TrackedSequence &ts, size_t frame1, size_t frame2,
		size_t ransac_rounds,
		double *error_thresh
		);

} // namespace mv

#endif // _ROBUST_SIXPOINTNVIEW_H

ADD_SUBDIRECTORY( testTrackedSequence )


INCLUDE_DIRECTORIES(
	${VXL_OXL_INCLUDE_DIR}
)



ADD_LIBRARY( MVsparse
	Frame.cc
	PointFeature.cc
	PointMatch.cc
	PointTrack.cc
	SiftFeature.cc
	TrackedSequence.cc
	TwoViewMatches.cc
)

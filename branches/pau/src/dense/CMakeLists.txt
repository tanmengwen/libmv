ADD_SUBDIRECTORY( shaders )
ADD_SUBDIRECTORY( tests )

ADD_LIBRARY( MVdense
	Camera.cc
	Image.cc
	MiddleburyData.cc
	PlaneSweepStereo.cc
	PlaneSweepStereoGPU.cc
)

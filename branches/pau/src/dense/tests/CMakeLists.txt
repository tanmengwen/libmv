
ADD_EXECUTABLE( test_Camera test_Camera.cc )
TARGET_LINK_LIBRARIES( test_Camera
			MVdense MVopengl
			vcl vnl vil png jpeg tiff
)


ADD_EXECUTABLE( test_MiddleburyData test_MiddleburyData.cc )
TARGET_LINK_LIBRARIES( test_MiddleburyData
			MVdense MVopengl
			vcl vnl vil png jpeg tiff
)

ADD_EXECUTABLE( test_PlaneSweepStereoGPU test_PlaneSweepStereoGPU.cc )
TARGET_LINK_LIBRARIES( test_PlaneSweepStereoGPU
			MVdense MVopengl
			vcl vnl vil png jpeg tiff
			${OPENGL_LIBRARIES})





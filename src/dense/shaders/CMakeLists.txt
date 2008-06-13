


MACRO(ADD_SHADER shadername)
	ADD_CUSTOM_TARGET( ${shadername}.h ALL
			COMMAND ${mv_SOURCE_DIR}/../scripts/cc_from_shader.py ${shadername}
			DEPENDS ${shadername}.vert ${shadername}.frag
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
ENDMACRO(ADD_SHADER)




ADD_SHADER(PlaneSweepShader)
ADD_SHADER(WarpImageShader)
ADD_SHADER(WarpImageSDShader)
ADD_SHADER(WarpImageNCCShader)
ADD_SHADER(AgregateScoreZNCCShader)
ADD_SHADER(GrayScaleAndSquareShader)
ADD_SHADER(PushInSortedListShader)
ADD_SHADER(UpdateBestDepthShader)
ADD_SHADER(ConvolutionShader)
ADD_SHADER(ConvolutionRGBAShader)

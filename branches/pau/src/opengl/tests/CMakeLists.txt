
ADD_EXECUTABLE( test_Pbuffer test_Pbuffer.cc )
TARGET_LINK_LIBRARIES( test_Pbuffer
			MVopengl
			vil png jpeg tiff
			${OPENGL_LIBRARIES})

ADD_EXECUTABLE( test_FramebufferObject test_FramebufferObject.cc )
TARGET_LINK_LIBRARIES( test_FramebufferObject
			MVopengl
			vil png jpeg tiff
			${OPENGL_LIBRARIES})


ADD_EXECUTABLE( test_ShaderProgram test_ShaderProgram.cc )
TARGET_LINK_LIBRARIES( test_ShaderProgram
			MVopengl
			vil png jpeg tiff
			${OPENGL_LIBRARIES})

ADD_CUSTOM_TARGET(
  ${shadername}.h ALL
  COMMAND ${mv_SOURCE_DIR}/../scripts/cc_from_shader.py TestShader
  DEPENDS TestShader.vert TestShader.frag
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})


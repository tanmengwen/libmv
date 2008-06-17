SUBDIRS( tests )

INCLUDE_DIRECTORIES(extern)

ADD_LIBRARY( MVopengl
	FramebufferObject.cc
	opengl.cc
	Pbuffer.cc
	ShaderProgram.cc
	extern/glew.c
)

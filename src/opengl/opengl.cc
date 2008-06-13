
#include "opengl.h"

bool opengl_init()
{
	// init glew and test for OpenGL 2.0
	glewInit();
	if(!glewIsSupported("GL_VERSION_2_0")) {
		printf("OpenGL 2.0 not supported\n");
		return false;
	}
	return true;
}


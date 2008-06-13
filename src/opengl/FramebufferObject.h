#ifndef __FramebufferObject_h__
#define __FramebufferObject_h__

#include "opengl.h"
#include <vcl_iostream.h>

class FramebufferObject
{
	GLuint framebufferID;
	GLuint prev_framebufferID;

	GLuint RGBbuffer;
	GLuint depthbuffer;
	int width,height;

public:
	FramebufferObject()
	{
		framebufferID = 0;
		prev_framebufferID = 0;
		RGBbuffer = 0;
		depthbuffer = 0;
		width = 0;
		height = 0;
	}

	bool init(int w,int h)
	{
		width = w;
		height = h;

		if(!opengl_init())
			return false;

		// get a name for the object
		glGenFramebuffersEXT(1,&framebufferID);
	}

	void create_depth_renderbuffer()
	{
		begin();

		// create a depth buffer
		glGenRenderbuffersEXT(1, &depthbuffer);
		// and select it
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);

		// allocate pixels
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
					 width, height);
		// attach the the renderbuffer to the framebuffer
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
					     GL_RENDERBUFFER_EXT, depthbuffer);

		end();
	}

	void create_color_renderbuffer()
	{
		begin();

		// create a renderbuffer
		glGenRenderbuffersEXT(1, &RGBbuffer);
		// and select it
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, RGBbuffer);
		// allocate pixels
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB,
					 width, height);
		// attach the renderbuffer to the framebuffer
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
					     GL_RENDERBUFFER_EXT, RGBbuffer);

		end();
	}

	void attach_color_texture(GLuint texID)
	{
		begin();

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
					  GL_TEXTURE_2D, texID, 0);

		end();
	}

	void attach_depth_texture(GLuint texID)
	{
		begin();

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
		       GL_TEXTURE_2D, texID, 0);

		end();
	}

	void begin() {
		GLint tmp;
		glGetIntegerv( GL_FRAMEBUFFER_BINDING_EXT, &tmp );
		prev_framebufferID = tmp;
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, framebufferID);
	}

	void end() {
		glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, prev_framebufferID );
		prev_framebufferID = 0;
	}

	void release() {
		glDeleteRenderbuffersEXT(1, &RGBbuffer);
		glDeleteRenderbuffersEXT(1, &depthbuffer);
		glDeleteFramebuffersEXT(1, &framebufferID);
	}
};


#endif //__FrameBufferObject_h__

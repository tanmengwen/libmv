#ifndef __Pbuffer_h__
#define __Pbuffer_h__


extern "C"
{
#include <stdio.h>

#include "opengl.h"
#include <GL/glx.h>
}

// TODO port this to OSx and Windows
class Pbuffer
{
protected:
	Display *display;
	GLXPbuffer glxPbuffer;
	GLXContext glxContext;

	Display *olddisplay;
	GLXDrawable oldglxDrawable;
	GLXContext oldglxContext;

public:
	Pbuffer() {
		display=0;
		glxPbuffer=0;
		glxContext=0;
	}


	bool init(int width, int height) {
		// get a display
		if(!display) display = XOpenDisplay(0);
		if(!display) {
			fprintf(stderr, "OffScreenGL: unable to get a X display.\n");
			return false;
		}

		// get the list of suitables configs
		int FBAttribList[] = {  GLX_DRAWABLE_TYPE, GLX_PBUFFER_BIT,
					GLX_RENDER_TYPE, GLX_RGBA_BIT,
					GLX_BUFFER_SIZE,4*8,
					GLX_ALPHA_SIZE,8,
					GLX_DEPTH_SIZE,16,
					GLX_DOUBLEBUFFER,false,
					0 };
		int iConfigCount;
		GLXFBConfig *glxConfigs = glXChooseFBConfig(display, 0, FBAttribList, &iConfigCount);
		if (!glxConfigs) {
			fprintf(stderr, "pbuffer creation error:  glXChooseFBConfig() failed\n");
			return false;
		}
		// Keep the first avaible config
		GLXFBConfig glxConfig = glxConfigs[0];
		XFree(glxConfigs);

		// Create a pbuffer
		int PBAttribList[] = {
			GLX_PBUFFER_WIDTH, width,
			GLX_PBUFFER_HEIGHT, height,
			GLX_LARGEST_PBUFFER, true,
			GLX_PRESERVED_CONTENTS, true,
			0 };
		glxPbuffer = glXCreatePbuffer(display, glxConfig, PBAttribList);
		if (!glxPbuffer) {
			fprintf(stderr, "pbuffer creation error:  glXCreatePbuffer() failed\n");
			return false;
		}

		// Create an openGL context to write into the pbuffer
		glxContext = glXCreateNewContext(display, glxConfig, GLX_RGBA_TYPE, NULL, true);
		if (!glxContext) {
			fprintf(stderr, "pbuffer creation error:  glXCreateNewContext() failed\n");
			return false;
		}

		return true;
	}

	void release() {
		if(glxPbuffer) {
			glXDestroyPbuffer(display,glxPbuffer);
			glxPbuffer=0;
		}
		if(glxContext) {
			glXDestroyContext(display,glxContext);
			glxContext=0;
		}
		if(display) {
			XCloseDisplay(display);
			display=0;
		}
	}

	bool begin() {
		olddisplay = glXGetCurrentDisplay();
		oldglxDrawable = glXGetCurrentDrawable();
		oldglxContext = glXGetCurrentContext();

		if(!glXMakeCurrent(display, glxPbuffer, glxContext)) {
			fprintf(stderr, "glXMakeCurrent failed at begin().\n");
			return false;
		}
		return true;
	}
	bool end() {
		if(olddisplay != 0 && oldglxDrawable != 0 && oldglxContext != 0)
			if(!glXMakeCurrent(olddisplay, oldglxDrawable, oldglxContext)) {
				fprintf(stderr, "glXMakeCurrent failed at end().\n");
				return false;
			}
		olddisplay = 0;
		oldglxDrawable = 0;
		oldglxContext = 0;
		return true;
	}

	void debug_info() {
		unsigned int w, h;
		w = h = 0;
		glXQueryDrawable(display, glxPbuffer, GLX_WIDTH, &w);
		glXQueryDrawable(display, glxPbuffer, GLX_HEIGHT, &h);

		printf( "(%d,%d)\n",w,h );
	}
};



	//~ void glut_trick() {
		//~ // init glut in order to get a display
		//~ int argc=1;
		//~ char *argv[]={"caca"};
		//~ glutInit(&argc, argv);
		//~ glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH);
		//~ glutInitWindowSize(3, 3);
		//~ glutInitWindowPosition(0, 0);
		//~ int window = glutCreateWindow("Toy window that should never appear on screen");
		//~ glutDestroyWindow(window);
	//~ }
	//~ bool init_display() {
		//~ // get the current display
		//~ //display = glXGetCurrentDisplay();
		//~ display = XOpenDisplay(0);

		//~ if (!display) {
			//~ printf("glut trickkkkk\n");
			//~ glut_trick();
			//~ display = glXGetCurrentDisplay();
			//~ if (!display) {
				//~ fprintf(stderr, "no display found by glXGetCurrentDisplay()\n");
				//~ return false;
			//~ }
		//~ }
		//~ iScreen = DefaultScreen(display);
		//~ return true;
	//~ }

#endif //__Pbuffer_h__


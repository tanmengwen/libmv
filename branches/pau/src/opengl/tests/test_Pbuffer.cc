
#include "../Pbuffer.h"

#include <vil/vil_image_view.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>

void save_framebuffer(const char *filename, int width, int height)
{
	vil_image_view<vxl_byte> tmp(width,height,1,3);
	glReadPixels( 0,0, width,height, GL_RGB,GL_UNSIGNED_BYTE, tmp.top_left_ptr());
	vil_save(tmp,filename);
}

int main()
{
	vil_image_view<vxl_byte> im;
	im = vil_load("/home/paulinus/pro/mv/src/dense/tests/data/templeR0043.png");

	int width=320,height=240;

	Pbuffer pb;
	pb.init(width,height);
	pb.begin();
	{
		// load texture
		GLuint textureID;
		glGenTextures(1, &textureID);

		// select the texture
		glBindTexture(GL_TEXTURE_2D, textureID);

		// enable bilinear filtering
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

		glTexImage2D( GL_TEXTURE_2D, 0, 3, im.ni(), im.nj(),
			      0, GL_RGB, GL_UNSIGNED_BYTE, im.top_left_ptr() );

		// opengl setup
		glViewport(0,0,width,height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-1,1,-1,1,0,10);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, textureID);
		glBegin(GL_QUADS);
		glColor3f(1,1,1);
		glTexCoord2f(0,0); glVertex3d(-1,-1, 0);
		glTexCoord2f(1,0); glVertex3d( 1,-1, 0);
		glTexCoord2f(1,1); glVertex3d( 1, 1, 0);
		glTexCoord2f(0,1); glVertex3d(-1, 1, 0);
		glEnd();

		save_framebuffer("/home/paulinus/Desktop/test_Pbuffer.png",width,height);
	}
	// release the pbuffer
	pb.end();
	pb.release();
	return 0;
}


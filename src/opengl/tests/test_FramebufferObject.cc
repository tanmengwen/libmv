
#include "../FramebufferObject.h"
#include "../Pbuffer.h"

#include <vil/vil_image_view.h>
#include <vil/vil_save.h>


int main()
{
	int pbwidth=640,pbheight=480;
	int texwidth=400,texheight=400;

	// Create a pbuffer to render the final image
	Pbuffer pb;
	pb.init(pbwidth,pbheight);
	pb.begin();
	{

		// create a color texture
		GLuint textureID;
		glGenTextures(1, &textureID);
		// select the texture
		glBindTexture(GL_TEXTURE_2D, textureID);
		// allocate pixels
		glTexImage2D( GL_TEXTURE_2D, 0, 3, texwidth, texheight,
			0, GL_RGB, GL_FLOAT, NULL );
		// enable bilinear filtering
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );


		////////////////////////////////
		// render to texture
		FramebufferObject fb;
		fb.init(texwidth,texheight);
		fb.attach_color_texture(textureID);
		fb.create_depth_renderbuffer();
		fb.begin();
		{
			// opengl setup
			glViewport(0,0,texwidth,texheight);
			glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
			glBegin(GL_TRIANGLES);
			glColor3f(1,0,1);
			glVertex3d(-.8,-.8, 0);
			glVertex3d( .8,-.8, 0);
			glVertex3d( .8, .8, 0);
			glEnd();
		}
		fb.end();
		fb.release();

		////////////////////////////////
		// use the texture
		glViewport(0,0,pbwidth,pbheight);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, textureID);
		glBegin(GL_QUADS);
		glColor3f(1,1,1);
		glTexCoord2f(0,0); glVertex3d( -1, -1, 0);
		glTexCoord2f(1,0); glVertex3d(-.5, -1, 0);
		glTexCoord2f(1,1); glVertex3d(-.5,-.5, 0);
		glTexCoord2f(0,1); glVertex3d( -1,-.5, 0);

		glTexCoord2f(0,0); glVertex3d(  1,  1, 0);
		glTexCoord2f(1,0); glVertex3d( .5,  1, 0);
		glTexCoord2f(1,1); glVertex3d( .5, .5, 0);
		glTexCoord2f(0,1); glVertex3d(  1, .5, 0);
		glEnd();


		// Read data from framebuffer
		vil_image_view<vxl_byte> tmp(pbwidth,pbheight,1,3);
		glReadPixels( 0,0, pbwidth,pbheight, GL_RGB,GL_UNSIGNED_BYTE, tmp.top_left_ptr() );
		vil_save(tmp,"/home/paulinus/Desktop/test_FramebufferObject.png");

	}
	pb.end();
	pb.release();

	return 0;
}


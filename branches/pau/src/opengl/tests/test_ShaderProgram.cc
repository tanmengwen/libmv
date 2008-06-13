
#include "../ShaderProgram.h"
#include "../Pbuffer.h"

#include <vil/vil_image_view.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>

#include "TestShader.h"

int main()
{
	vil_image_view<vxl_byte> im;
	im = vil_load("/home/paulinus/pro/mv/src/dense/tests/data/templeR0043.png");
	vil_image_view<float> fim(im.ni(),im.nj(),1,im.nplanes());
	vil_convert_cast(im,fim);

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

		// load the image data
		for(uint i=0; i<fim.size(); i++)
			fim.top_left_ptr()[i] /= 255.0f;

		glTexImage2D( GL_TEXTURE_2D, 0, 3, fim.ni(), fim.nj(),
			      0, GL_RGB, GL_FLOAT, fim.top_left_ptr() );


		// opengl setup
		glViewport(0,0,width,height);

		// load and activate the shader
		ShaderProgram sp;
		sp.init(TestShader_vert, TestShader_frag);
		sp.begin();
		{
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, textureID);

			glUniform1i(sp.get_uniform("templeTex"),0);

			glBegin(GL_QUADS);
			glColor3f(1,1,1);
			glTexCoord2f(0,0); glVertex3d(-.9,-.9, 0);
			glTexCoord2f(1,0); glVertex3d( .9,-.9, 0);
			glTexCoord2f(1,1); glVertex3d( .9, .9, 0);
			glTexCoord2f(0,1); glVertex3d(-.9, .9, 0);
			glEnd();
		}
		sp.end();
		sp.release();


		// Read data from framebuffer
		vil_image_view<vxl_byte> tmp(width,height,1,3);
		vil_image_view<float> ftmp(width,height,1,3);
		glReadPixels( 0,0, width,height, GL_RGB,GL_FLOAT, ftmp.top_left_ptr() );

		for(int i=0; i<ftmp.size(); i++)
			ftmp.top_left_ptr()[i] *= 255;

		vil_convert_cast(ftmp,tmp);
		vil_save(tmp,"/home/paulinus/Desktop/test_ShaderProgram.png");
	}
	// release the pbuffer
	pb.end();
	pb.release();
	return 0;
}


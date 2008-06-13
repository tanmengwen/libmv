
#include "PlaneSweepStereoGPU.h"


#include <vil/vil_save.h>

#include <stdarg.h>

#include "PlaneSweepStereo.h"
#include "../opengl/opengl.h"
#include "../opengl/Pbuffer.h"
#include "../opengl/FramebufferObject.h"
#include "../opengl/ShaderProgram.h"

#include "shaders/PlaneSweepShader.h"
#include "shaders/WarpImageShader.h"
#include "shaders/WarpImageSDShader.h"
#include "shaders/WarpImageNCCShader.h"
#include "shaders/AgregateScoreZNCCShader.h"
#include "shaders/GrayScaleAndSquareShader.h"
#include "shaders/PushInSortedListShader.h"
#include "shaders/UpdateBestDepthShader.h"
#include "shaders/ConvolutionShader.h"
#include "shaders/ConvolutionRGBAShader.h"



PlaneSweepStereoGPU::PlaneSweepStereoGPU()
{
	current_best_depth_texture=0;
	current_depth_score_texture=0;
}


void PlaneSweepStereoGPU::load_image_texture(GLuint texID, Image &im)
{
	// select the texture
	glBindTexture(GL_TEXTURE_2D, texID);

	// enable bilinear filtering
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// load the image data
	float *data = new float[im.size()];

	for(uint p=0; p<im.size(); p++)
		data[p] = im.top_left_ptr()[p] / 255.0f;

	glTexImage2D( GL_TEXTURE_2D, 0, 3, im.ni(), im.nj(),
		0, GL_RGB, GL_FLOAT, data );

	delete [] data;
}


void PlaneSweepStereoGPU::load_image_textures()
{
	// get names
	glGenTextures(1, &key_texture);
	ref_textures.resize(nrefs());
	glGenTextures(nrefs(), &ref_textures[0]);

	load_image_texture(key_texture, key_image);
	for(int i=0; i<nrefs(); i++)
		load_image_texture(ref_textures[i],ref_images[i]);
}


void PlaneSweepStereoGPU::release_image_textures()
{
	glDeleteTextures(1,&key_texture);
	glDeleteTextures(nrefs(),&ref_textures[0]);
}


void PlaneSweepStereoGPU::allocate_score_depth_textures()
{
	// best depth
	glGenTextures(2,&best_depth_texture[0]);
	for(int i=0;i<2;i++) {
		glBindTexture(GL_TEXTURE_2D, best_depth_texture[i]);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
		glTexImage2D( GL_TEXTURE_2D, 0, 3, width(),height(),
			0, GL_RGB, GL_FLOAT, NULL );
	}

	// depth score
	glGenTextures(2,depth_score_texture);
	for(int i=0;i<2;i++) {
		glBindTexture(GL_TEXTURE_2D, depth_score_texture[i]);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
		glTexImage2D( GL_TEXTURE_2D, 0, 4, width(),height(),
			0, GL_RGBA, GL_FLOAT, NULL );
	}
	// warped image
	glGenTextures(1,&agregated_score_texture);
	glBindTexture(GL_TEXTURE_2D, agregated_score_texture);
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	glTexImage2D( GL_TEXTURE_2D, 0, 3, width(),height(),
		0, GL_RGB, GL_FLOAT, NULL );
	// warped image
	glGenTextures(1,&warped_image_texture);
	glBindTexture(GL_TEXTURE_2D, warped_image_texture);
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	glTexImage2D( GL_TEXTURE_2D, 0, 4, width(),height(),
		0, GL_RGBA, GL_FLOAT, NULL );
		// warped image
	glGenTextures(1,&tmp_texture);
	glBindTexture(GL_TEXTURE_2D, tmp_texture);
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	glTexImage2D( GL_TEXTURE_2D, 0, 4, width(),height(),
		0, GL_RGBA, GL_FLOAT, NULL );
}


void PlaneSweepStereoGPU::release_score_depth_textures()
{
	glDeleteTextures(2,best_depth_texture);
	glDeleteTextures(2,depth_score_texture);
	glDeleteTextures(1,&warped_image_texture);
	glDeleteTextures(1,&tmp_texture);
}


void PlaneSweepStereoGPU::clear_texture(GLuint texID,float r, float g, float b, float a)
{
	FramebufferObject fb;
	fb.init(width(),height());
	fb.attach_color_texture(texID);
	fb.begin();
	glClearColor(r,g,b,a);
	glClear(GL_COLOR_BUFFER_BIT);
	fb.end();
	fb.release();
}


void PlaneSweepStereoGPU::compute(int nplanes, CorrelationType ct, int kernel_size)
{
	// a dummy Pbuffer to get an opengl context
	Pbuffer pb;
	pb.init(16,16);
	pb.begin();
	{
		opengl_init();

		load_image_textures();
		allocate_score_depth_textures();

		new_compute(nplanes,ct,kernel_size);

		release_score_depth_textures();
		release_image_textures();
	}
	pb.end();
	pb.release();
}


void PlaneSweepStereoGPU::new_compute(int nplanes,CorrelationType ct, int kernel_size)
{
	near_and_far_from_bounding_box();
	vcl_vector<double> depths;
	get_plane_depths(depths,nplanes);

	// init best_depth and best_score
	for(int i=0;i<2;i++)
		clear_texture(best_depth_texture[i],99999,99999,99999,99999);

	if(ct==ZNCC)
		prepare_key_texture_ZNCC(kernel_size);

	// for every depth
	for(uint d=0;d<depths.size();d++)
	{
		// init the depth_scores buffer (it will contain the K best scores for this depth)
		for(int i=0;i<2;i++)
			clear_texture(depth_score_texture[i],99999,99999,99999,99999);
		// for every reference image
		for(int r=0;r<nrefs();r++)
		{
			// compute the warped image and/or the matching score w.r.t. the key image
			compute_warped_image(r,depths[d],ct);
//save_texture(warped_image_texture,"warped%d_%d.png",r,d);
			// agregate the score
			agregate_score(ct,kernel_size);
//save_texture(agregated_score_texture,"agregated%d_%d.png",r,d);
			// update the depth_scores buffer
			update_depth_score(r,depths[d]);
		}
		// update the best_depth and best_score buffers
		update_best_depth(depths[d]);
	}

//	save_texture(best_depth_texture[current_best_depth_texture],"/home/paulinus/Desktop/test_PlaneSweepStereo/best_depth.png");
	read_best_depth_texture();
}

void PlaneSweepStereoGPU::agregate_score(CorrelationType ct, int kernel_size)
{
	switch(ct) {
		case SD:   agregate_score_SD();   break;
		case SSD:  agregate_score_SSD(kernel_size);  break;
		case ZNCC: agregate_score_ZNCC(kernel_size); break;
	}
}

void PlaneSweepStereoGPU::read_best_depth_texture()
{
	depthmap = Image(width(),height(),1,3);

	glBindTexture(GL_TEXTURE_2D, best_depth_texture[current_best_depth_texture]);
	glGetTexImage(GL_TEXTURE_2D,0,GL_RGB,GL_FLOAT,depthmap.top_left_ptr());
}


Image PlaneSweepStereoGPU::get_depthmap()
{
	return depthmap;
}


vnl_float_3x3 PlaneSweepStereoGPU::image_to_image_homography_in_texture_coordinates(int r, float depth)
{
	vnl_double_4 plane = key_camera.frontoparallel_plane_from_depth(depth);

	// homography from the key image to the ref image in central pixel coordinates
	vnl_double_3x3 Him2im = Camera::H_from_2P_plane(key_camera, ref_cameras[r], plane);

	// from texture coordinates to central pixel coordinates of the key image
	vnl_double_3x3 Htex2pix = vnl_inverse(key_camera.H_tex_coords_from_pixel_coords());

	// from central pixel coordinates of the ref image to texture coordinates
	vnl_double_3x3 Hpix2tex = ref_cameras[r].H_tex_coords_from_pixel_coords();

	return vnl_float_3x3_from_x(Hpix2tex * Him2im * Htex2pix);
}


void PlaneSweepStereoGPU::compute_warped_image(int r, double depth, CorrelationType ct)
{
	const char *vertex_shader, *fragment_shader;
	switch(ct) {
		case SD: case SSD:
			vertex_shader = WarpImageSDShader_vert;
			fragment_shader = WarpImageSDShader_frag;
			break;
		case ZNCC:
			vertex_shader = WarpImageNCCShader_vert;
			fragment_shader = WarpImageNCCShader_frag;
			break;
	}


	FramebufferObject fb;
	fb.init(width(),height());
	fb.attach_color_texture(warped_image_texture);
	fb.begin();
	{
		glViewport(0,0,width(),height());
		place_opengl_camera();

		// set active textures
		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, key_texture);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, ref_textures[r]);

		// init the shader program
		ShaderProgram sp;
		sp.init(vertex_shader, fragment_shader);
		sp.begin();
		{
			// pass arguments to the program
			glUniform1i( sp.get_uniform("key_texture"), 0 );
			glUniform1i( sp.get_uniform("ref_texture"), 1 );

			// compute homography induced by a plane at depth depth
			vnl_float_3x3 H = image_to_image_homography_in_texture_coordinates(r,depth);
			glUniformMatrix3fv( sp.get_uniform("H"), 1, GL_TRUE, H.data_block() );

			// draw the plane
			vnl_double_3 tl = key_camera.unproject( -.5,-.5,depth);
			vnl_double_3 tr = key_camera.unproject(width()-.5,-.5,depth);
			vnl_double_3 bl = key_camera.unproject( -.5,height()-.5,depth);
			vnl_double_3 br = key_camera.unproject(width()-.5,height()-.5,depth);

			glBegin(GL_QUADS);
				glColor3f(1,1,1);
				glMultiTexCoord2f(GL_TEXTURE0,0,0);
				glVertex3dv(tl.data_block());

				glMultiTexCoord2f(GL_TEXTURE0,0,1);
				glVertex3dv(bl.data_block());

				glMultiTexCoord2f(GL_TEXTURE0,1,1);
				glVertex3dv(br.data_block());

				glMultiTexCoord2f(GL_TEXTURE0,1,0);
				glVertex3dv(tr.data_block());
			glEnd();

		} // release the shader program
		sp.end();
		sp.release();
	}
	fb.end();
	fb.release();
}


void PlaneSweepStereoGPU::agregate_score_SD()
{
	// TODO this should be done simply by copying the warped_texture_texture to the agregated_score_texture
	agregate_score_SSD(1);
}


void PlaneSweepStereoGPU::agregate_score_SSD(int kernel_size)
{
	float weights[kernel_size];
	float offsetx[kernel_size];
	float offsety[kernel_size];
	gaussian_convolution_kernel(weights,offsetx,offsety,kernel_size);

	// horitzontal blur
	convolve_texture( tmp_texture, warped_image_texture,    weights, offsetx,offsety, kernel_size );
	// vertical blur
	convolve_texture( agregated_score_texture, tmp_texture, weights, offsety,offsetx, kernel_size );
}

/// given a color texture, computes its gray scale (or intensity) version I and its square I*I
/// the intensity is stored on the red channel of dest_texture and the square on the green channel
void PlaneSweepStereoGPU::compute_grayscale_and_square(GLuint dest_texture, GLuint src_texture)
{
	FramebufferObject fb;
	fb.init(width(),height());
	fb.attach_color_texture(dest_texture);
	fb.begin();
	{
		// set active textures
		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, src_texture);

		// init the shader program
		ShaderProgram sp;
		sp.init(GrayScaleAndSquareShader_vert, GrayScaleAndSquareShader_frag);
		sp.begin();
		{
			// pass arguments to the program
			glUniform1i( sp.get_uniform("I_texture"), 0 );

			draw_canonical_rectangle();

		} // release the shader program
		sp.end();
		sp.release();
	}
	fb.end();
	fb.release();
}


// this function replaces the (R,G,B,A) colors of the key_texture by convoluted versions
// of its intensity
// the process is as follows
//      R         I           conv(I)
//      G   --\   I*I   --\   conv(I*I)
//      B   --/   1     --/   I
//      A         1           conv(1)
// note that blue channel contains the non-convoluted intensity image
void PlaneSweepStereoGPU::prepare_key_texture_ZNCC(int kernel_size)
{
	GLuint tmp2_texture = warped_image_texture;

	compute_grayscale_and_square(tmp_texture,key_texture);

	float weights[kernel_size];
	float offsetx[kernel_size];
	float offsety[kernel_size];
	gaussian_convolution_kernel(weights,offsetx,offsety,kernel_size);

	float weightsRGBA[kernel_size][4];
	for(int i=0;i<kernel_size;i++) {
		weightsRGBA[i][0] = weights[i];
		weightsRGBA[i][1] = weights[i];
		weightsRGBA[i][2] = ( i==(kernel_size-1)/2 )?1:0;
		weightsRGBA[i][3] = weights[i];
	}

	convolve_textureRGBA( tmp2_texture, tmp_texture, weightsRGBA,
			       offsetx,offsety, kernel_size );
	convolve_textureRGBA( key_texture, tmp2_texture, weightsRGBA,
			       offsety,offsetx, kernel_size );
}


void PlaneSweepStereoGPU::agregate_score_ZNCC(int kernel_size)
{
	// at this point
	// key_texture contains (conv(I), conv(I*I), I, conv(1))
	// and warped_image_texture contains (J, J*J, I*J, 1)
	float weights[kernel_size];
	float offsetx[kernel_size];
	float offsety[kernel_size];
	gaussian_convolution_kernel(weights,offsetx,offsety,kernel_size);

	convolve_texture( tmp_texture, warped_image_texture, weights, offsetx,offsety, kernel_size );
	convolve_texture( warped_image_texture, tmp_texture, weights, offsety,offsetx, kernel_size );


	// at this point
	// key_texture contains (conv(I), conv(I*I), I, conv(1))
	// and warped_image_texture contains (conv(J), conv(J*J), conv(I*J), conv(1))
	FramebufferObject fb;
	fb.init(width(),height());
	fb.attach_color_texture(agregated_score_texture);
	fb.begin();
	{
		// set active textures
		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, key_texture);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, warped_image_texture);

		// init the shader program
		ShaderProgram sp;
		sp.init(AgregateScoreZNCCShader_vert, AgregateScoreZNCCShader_frag);
		sp.begin();
		{
			// pass arguments to the program
			glUniform1i( sp.get_uniform("I_texture"), 0 );
			glUniform1i( sp.get_uniform("J_texture"), 1 );

			draw_canonical_rectangle();
		} // release the shader program
		sp.end();
		sp.release();
	}
	fb.end();
	fb.release();
}




void PlaneSweepStereoGPU::gaussian_convolution_kernel(float *weights,float *offsetsx, float *offsetsy, int kernel_size)
{
	// generated by mv/scripts/tartaglia.py and then copy-pasted here
	float weights1[1] = { 1 };
	float weights3[3] = { 0.25 , 0.5 , 0.25 };
	float weights5[5] = { 0.0625 , 0.25 , 0.375 , 0.25 , 0.0625 };
	float weights7[7] = { 0.015625 , 0.09375 , 0.234375 , 0.3125 , 0.234375 , 0.09375 , 0.015625 };
	float weights9[9] = { 0.00390625 , 0.03125 , 0.109375 , 0.21875 , 0.2734375 , 0.21875 , 0.109375 , 0.03125 , 0.00390625 };
	float weights11[11] = { 0.0009765625 , 0.009765625 , 0.0439453125 , 0.1171875 , 0.205078125 , 0.24609375 , 0.205078125 , 0.1171875 , 0.0439453125 , 0.009765625 , 0.0009765625 };
	float weights13[13] = { 0.000244140625 , 0.0029296875 , 0.01611328125 , 0.0537109375 , 0.120849609375 , 0.193359375 , 0.2255859375 , 0.193359375 , 0.120849609375 , 0.0537109375 , 0.01611328125 , 0.0029296875 , 0.000244140625 };
	float weights15[15] = { 6.103515625e-05 , 0.0008544921875 , 0.00555419921875 , 0.022216796875 , 0.0610961914062 , 0.122192382812 , 0.183288574219 , 0.20947265625 , 0.183288574219 , 0.122192382812 , 0.0610961914062 , 0.022216796875 , 0.00555419921875 , 0.0008544921875 , 6.103515625e-05 };
	float weights17[17] = { 1.52587890625e-05 , 0.000244140625 , 0.0018310546875 , 0.008544921875 , 0.0277709960938 , 0.066650390625 , 0.122192382812 , 0.174560546875 , 0.196380615234 , 0.174560546875 , 0.122192382812 , 0.066650390625 , 0.0277709960938 , 0.008544921875 , 0.0018310546875 , 0.000244140625 , 1.52587890625e-05 };
	float weights19[19] = { 3.81469726562e-06 , 6.86645507812e-05 , 0.000583648681641 , 0.00311279296875 , 0.0116729736328 , 0.0326843261719 , 0.0708160400391 , 0.121398925781 , 0.166923522949 , 0.185470581055 , 0.166923522949 , 0.121398925781 , 0.0708160400391 , 0.0326843261719 , 0.0116729736328 , 0.00311279296875 , 0.000583648681641 , 6.86645507812e-05 , 3.81469726562e-06 };
	float weights21[21] = { 9.53674316406e-07 , 1.90734863281e-05 , 0.000181198120117 , 0.0010871887207 , 0.00462055206299 , 0.0147857666016 , 0.0369644165039 , 0.0739288330078 , 0.120134353638 , 0.160179138184 , 0.176197052002 , 0.160179138184 , 0.120134353638 , 0.0739288330078 , 0.0369644165039 , 0.0147857666016 , 0.00462055206299 , 0.0010871887207 , 0.000181198120117 , 1.90734863281e-05 , 9.53674316406e-07 };
	float weights23[23] = { 2.38418579102e-07 , 5.24520874023e-06 , 5.50746917725e-05 , 0.000367164611816 , 0.00174403190613 , 0.00627851486206 , 0.0177891254425 , 0.0406608581543 , 0.0762391090393 , 0.118594169617 , 0.154172420502 , 0.168188095093 , 0.154172420502 , 0.118594169617 , 0.0762391090393 , 0.0406608581543 , 0.0177891254425 , 0.00627851486206 , 0.00174403190613 , 0.000367164611816 , 5.50746917725e-05 , 5.24520874023e-06 , 2.38418579102e-07 };
	float weights25[25] = { 5.96046447754e-08 , 1.43051147461e-06 , 1.6450881958e-05 , 0.000120639801025 , 0.000633358955383 , 0.00253343582153 , 0.00802254676819 , 0.0206294059753 , 0.0438374876976 , 0.0779333114624 , 0.116899967194 , 0.148781776428 , 0.161180257797 , 0.148781776428 , 0.116899967194 , 0.0779333114624 , 0.0438374876976 , 0.0206294059753 , 0.00802254676819 , 0.00253343582153 , 0.000633358955383 , 0.000120639801025 , 1.6450881958e-05 , 1.43051147461e-06 , 5.96046447754e-08 };
	float weights27[27] = { 1.49011611938e-08 , 3.8743019104e-07 , 4.842877388e-06 , 3.8743019104e-05 , 0.000222772359848 , 0.000980198383331 , 0.00343069434166 , 0.00980198383331 , 0.0232797116041 , 0.0465594232082 , 0.079151019454 , 0.115128755569 , 0.143910944462 , 0.154981017113 , 0.143910944462 , 0.115128755569 , 0.079151019454 , 0.0465594232082 , 0.0232797116041 , 0.00980198383331 , 0.00343069434166 , 0.000980198383331 , 0.000222772359848 , 3.8743019104e-05 , 4.842877388e-06 , 3.8743019104e-07 , 1.49011611938e-08 };
	float weights29[29] = { 3.72529029846e-09 , 1.04308128357e-07 , 1.40815973282e-06 , 1.22040510178e-05 , 7.6275318861e-05 , 0.000366121530533 , 0.00140346586704 , 0.00441089272499 , 0.0115785934031 , 0.0257302075624 , 0.0488873943686 , 0.0799975544214 , 0.113329868764 , 0.139482915401 , 0.149445980787 , 0.139482915401 , 0.113329868764 , 0.0799975544214 , 0.0488873943686 , 0.0257302075624 , 0.0115785934031 , 0.00441089272499 , 0.00140346586704 , 0.000366121530533 , 7.6275318861e-05 , 1.22040510178e-05 , 1.40815973282e-06 , 1.04308128357e-07 , 3.72529029846e-09 };
	float weights31[31] = { 9.31322574615e-10 , 2.79396772385e-08 , 4.05125319958e-07 , 3.78116965294e-06 , 2.55228951573e-05 , 0.000132719054818 , 0.000552996061742 , 0.0018959864974 , 0.00545096118003 , 0.0133245717734 , 0.0279816007242 , 0.0508756376803 , 0.0805530929938 , 0.111535051838 , 0.135435420088 , 0.144464448094 , 0.135435420088 , 0.111535051838 , 0.0805530929938 , 0.0508756376803 , 0.0279816007242 , 0.0133245717734 , 0.00545096118003 , 0.0018959864974 , 0.000552996061742 , 0.000132719054818 , 2.55228951573e-05 , 3.78116965294e-06 , 4.05125319958e-07 , 2.79396772385e-08 , 9.31322574615e-10 };
	float weights33[33] = { 2.32830643654e-10 , 7.45058059692e-09 , 1.15483999252e-07 , 1.15483999252e-06 , 8.37258994579e-06 , 4.68865036964e-05 , 0.000210989266634 , 0.000783674418926 , 0.00244898255914 , 0.00653062015772 , 0.0150204263628 , 0.0300408527255 , 0.0525714922696 , 0.0808792188764 , 0.109764654189 , 0.131717585027 , 0.139949934091 , 0.131717585027 , 0.109764654189 , 0.0808792188764 , 0.0525714922696 , 0.0300408527255 , 0.0150204263628 , 0.00653062015772 , 0.00244898255914 , 0.000783674418926 , 0.000210989266634 , 4.68865036964e-05 , 8.37258994579e-06 , 1.15483999252e-06 , 1.15483999252e-07 , 7.45058059692e-09 , 2.32830643654e-10 };
	float weights35[35] = { 5.82076609135e-11 , 1.97906047106e-09 , 3.26544977725e-08 , 3.48314642906e-07 , 2.69943848252e-06 , 1.61966308951e-05 , 7.82837159932e-05 , 0.000313134863973 , 0.00105683016591 , 0.00305306492373 , 0.00763266230933 , 0.0166530814022 , 0.0319184060208 , 0.0540157640353 , 0.0810236460529 , 0.108031528071 , 0.128287439584 , 0.135833759559 , 0.128287439584 , 0.108031528071 , 0.0810236460529 , 0.0540157640353 , 0.0319184060208 , 0.0166530814022 , 0.00763266230933 , 0.00305306492373 , 0.00105683016591 , 0.000313134863973 , 7.82837159932e-05 , 1.61966308951e-05 , 2.69943848252e-06 , 3.48314642906e-07 , 3.26544977725e-08 , 1.97906047106e-09 , 5.82076609135e-11 };

	float *w;
	switch(kernel_size)
	{
		case 1 : w = weights1; break;
		case 3 : w = weights3; break;
		case 5 : w = weights5; break;
		case 7 : w = weights7; break;
		case 9 : w = weights9; break;
		case 11: w = weights11; break;
		case 13: w = weights13; break;
		case 15: w = weights15; break;
		case 17: w = weights17; break;
		case 19: w = weights19; break;
		case 21: w = weights21; break;
		case 23: w = weights23; break;
		case 25: w = weights25; break;
		case 27: w = weights27; break;
		case 29: w = weights29; break;
		case 31: w = weights31; break;
		case 33: w = weights33; break;
		case 35: w = weights35; break;
	}

	for(int i=0; i<kernel_size; i++) {
		weights[i] = w[i];
		offsetsx[i] = i - (kernel_size-1)/2;
		offsetsy[i] = 0;
	}
}



void PlaneSweepStereoGPU::convolve_texture(GLuint dest, GLuint source,
		float weights[], float offsets_x[], float offsets_y[], int kernel_size)
{
	vcl_vector<float> normalized_offsets;
	normalized_offsets.resize(2*kernel_size);
	for(int i=0; i<kernel_size; i++) {
		normalized_offsets[2*i] = offsets_x[i] / width();
		normalized_offsets[2*i+1] = offsets_y[i] / width();
	}

	FramebufferObject fb;
	fb.init(width(),height());
	fb.attach_color_texture(dest);
	fb.begin();
	{
		// set active textures
		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, source);

		// init the shader program
		ShaderProgram sp;
		sp.init(ConvolutionShader_vert, ConvolutionShader_frag);
		sp.begin();
		{
			// pass arguments to the program
			glUniform1i( sp.get_uniform("source"), 0 );
			glUniform2fv( sp.get_uniform("offsets"),kernel_size, &normalized_offsets[0] );
			glUniform1fv( sp.get_uniform("weights"),kernel_size, &weights[0] );
			glUniform1i( sp.get_uniform("kernel_size"), kernel_size);

			draw_canonical_rectangle();

		} // release the shader program
		sp.end();
		sp.release();
	}
	fb.end();
	fb.release();
}

void PlaneSweepStereoGPU::convolve_textureRGBA(GLuint dest, GLuint source,
					   float weightsRGBA[][4], float offsets_x[], float offsets_y[], int kernel_size)
{
	vcl_vector<float> normalized_offsets;
	normalized_offsets.resize(2*kernel_size);
	for(int i=0; i<kernel_size; i++) {
		normalized_offsets[2*i] = offsets_x[i] / width();
		normalized_offsets[2*i+1] = offsets_y[i] / width();
	}

	FramebufferObject fb;
	fb.init(width(),height());
	fb.attach_color_texture(dest);
	fb.begin();
	{
		// set active textures
		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, source);

		// init the shader program
		ShaderProgram sp;
		sp.init(ConvolutionRGBAShader_vert, ConvolutionRGBAShader_frag);
		sp.begin();
		{
			// pass arguments to the program
			glUniform1i( sp.get_uniform("source"), 0 );
			glUniform2fv( sp.get_uniform("offsets"),kernel_size, &normalized_offsets[0] );
			glUniform4fv( sp.get_uniform("weights"),kernel_size, &weightsRGBA[0][0] );
			glUniform1i( sp.get_uniform("kernel_size"), kernel_size);

			draw_canonical_rectangle();

		} // release the shader program
		sp.end();
		sp.release();
	}
	fb.end();
	fb.release();
}

void PlaneSweepStereoGPU::update_depth_score(int r, float depth)
{
	// swap buffers
	current_depth_score_texture = 1-current_depth_score_texture;

	FramebufferObject fb;
	fb.init(width(),height());
	fb.attach_color_texture(depth_score_texture[current_depth_score_texture]);
	fb.begin();
	{
		// set active textures
		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, agregated_score_texture);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, depth_score_texture[1-current_depth_score_texture]);

		// init the shader program
		ShaderProgram sp;
		sp.init(PushInSortedListShader_vert, PushInSortedListShader_frag);
		sp.begin();
		{
			// pass arguments to the program
			glUniform1i( sp.get_uniform("to_push_texture"), 0 );
			glUniform1i( sp.get_uniform("sorted_list_texture"), 1 );

			draw_canonical_rectangle();

		} // release the shader program
		sp.end();
		sp.release();
	}
	fb.end();
	fb.release();


//	save_texture( depth_score_texture[current_depth_score_texture],
//		      "/home/paulinus/Desktop/test_PlaneSweepStereo/depth_score%04d_%f.png",r,depth);
}


void PlaneSweepStereoGPU::update_best_depth(float depth)
{
	// swap buffers
	current_best_depth_texture = 1-current_best_depth_texture;

	FramebufferObject fb;
	fb.init(width(),height());
	fb.attach_color_texture(best_depth_texture[current_best_depth_texture]);
	fb.begin();
	{
		// set active textures
		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, depth_score_texture[current_depth_score_texture]);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, best_depth_texture[1-current_best_depth_texture]);

		// init the shader program
		ShaderProgram sp;
		sp.init(UpdateBestDepthShader_vert, UpdateBestDepthShader_frag);
		sp.begin();
		{
			// pass arguments to the program
			glUniform1i( sp.get_uniform("depth_score_texture"), 0 );
			glUniform1i( sp.get_uniform("best_depth_texture"), 1 );
			glUniform1f( sp.get_uniform("depth"), depth );

			draw_canonical_rectangle();

		} // release the shader program
		sp.end();
		sp.release();
	}
	fb.end();
	fb.release();


//		save_texture( best_depth_texture[current_best_depth_texture],
//			      "/home/paulinus/Desktop/test_PlaneSweepStereo/best_depth%f.png",depth);
}


void PlaneSweepStereoGPU::draw_canonical_rectangle()
{
	glViewport(0,0,width(),height());
	glMatrixMode(GL_PROJECTION); glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);  glLoadIdentity();

	glBegin(GL_QUADS);
	glColor3f(1,1,1);
	glMultiTexCoord2f(GL_TEXTURE0,0,0);
	glVertex3d(-1,-1,0);

	glMultiTexCoord2f(GL_TEXTURE0,0,1);
	glVertex3d(-1, 1,0);

	glMultiTexCoord2f(GL_TEXTURE0,1,1);
	glVertex3d( 1, 1,0);

	glMultiTexCoord2f(GL_TEXTURE0,1,0);
	glVertex3d( 1,-1,0);
	glEnd();
}


void PlaneSweepStereoGPU::save_texture(GLuint texID, const char *filename, ... )
{
	vil_image_view<vxl_byte> tmp(width(),height(),1,3);
	glBindTexture(GL_TEXTURE_2D, texID);
	glGetTexImage(GL_TEXTURE_2D,0,GL_RGB,GL_UNSIGNED_BYTE,tmp.top_left_ptr());
	vcl_cout << glGetError() << " " << GL_NO_ERROR << vcl_endl;

	va_list argp;
	va_start(argp,filename);
	char buf[300];
	vsprintf(buf,filename,argp);
	va_end(argp);
	vil_save(tmp,buf);
}


void PlaneSweepStereoGPU::draw_bounding_box()
{
	glBegin(GL_LINES);
	glColor3f(0,1.,0);
	for(int x=0;x<2;x++) for(int y=0;y<2;y++) for(int z=0;z<2;z++)
		glVertex3d( bbox[x][0], bbox[y][1], bbox[z][2] );
	for(int y=0;y<2;y++) for(int z=0;z<2;z++) for(int x=0;x<2;x++)
		glVertex3d( bbox[x][0], bbox[y][1], bbox[z][2] );
	for(int z=0;z<2;z++) for(int x=0;x<2;x++) for(int y=0;y<2;y++)
		glVertex3d( bbox[x][0], bbox[y][1], bbox[z][2] );
	glEnd();
}


void PlaneSweepStereoGPU::place_opengl_camera()
{
	// recover opengl matrices from the camera
	double glP[4][4],glM[4][4];
	key_camera.openGLprojection(glP);
	key_camera.openGLmodelview(glM);

	// set the matrices
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixd(&glP[0][0]);
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixd(&glM[0][0]);
}




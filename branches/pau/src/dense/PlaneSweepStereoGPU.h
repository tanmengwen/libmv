#ifndef __PlaneSweepStereoGPU_h__
#define __PlaneSweepStereoGPU_h__

#include <vnl/vnl_float_3x3.h>
#include <vnl/vnl_float_3.h>

#include "PlaneSweepStereo.h"
#include "../opengl/opengl.h"
#include "../opengl/ShaderProgram.h"
#include "../opengl/FramebufferObject.h"



template<class T>
vnl_float_3x3 vnl_float_3x3_from_x( const T &x ) {
  vnl_float_3x3 res;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      res(i,j) = x(i,j);
  return res;
}
template<class T>
vnl_float_3 vnl_float_3_from_x( const T &x ) {
  vnl_float_3 res;
  for(int i = 0; i < 3; ++i)
    res(i) = x(i);
  return res;
}



class PlaneSweepStereoGPU : public PlaneSweepStereo {
public:
  PlaneSweepStereoGPU();
  int AddFrontoParallelPlanes(int nplanes);
  int AddPlane(const vnl_double_4 &plane);
  void compute(int nplanes, CorrelationType ct, int kernel_size);
  Image get_depthmap();
  Image get_confidence();

private:
  void init_shaders();
  void release_shaders();
  void init_framebuffers();
  void release_framebuffers();

  void load_image_texture(GLuint texID, Image &im);
  void load_image_textures();
  void release_image_textures();
  void allocate_score_depth_textures();
  void release_score_depth_textures();

  void new_compute(int nplanes, CorrelationType ct, int kernel_size);

  void prepare_key_texture_ZNCC(int key_texture);
  void compute_grayscale_and_square(GLuint src_texture,
                                    FramebufferObject *dest_fbo);

  void compute_warped_image(int r, int plane_index, CorrelationType ct);

  void agregate_score(CorrelationType ct, int kernel_size);
  void agregate_score_SD();
  void agregate_score_SSD(int kernel_size);
  void agregate_score_ZNCC(int kernel_size);

  void update_depth_score(int r);
  void update_best_depth(int plane_number);
  void read_best_depth_texture();

  vnl_float_3x3 ImagePlaneImageHomographyInTexCoords(int r,
                                                     const vnl_double_4 &plane);

  void clear_framebuffer(float r, float g, float b, float a,
                         FramebufferObject *fbo);
  void save_texture(GLuint texID, const char *filename, ... );
  void convolve_texture(GLuint source,
                        float weights[],
                        float offsets_x[],
                        float offsets_y[],
                        int kernel_size,
                        FramebufferObject *dest_fbo);
  void convolve_textureRGBA(GLuint source,
                            float weightsRGBA[][4],
                            float offsets_x[],
                            float offsets_y[],
                            int kernel_size,
                            FramebufferObject *dest_fbo);
  void gaussian_convolution_kernel(float *weights,
                                   float *offsetsx,
                                   float *offsetsy,
                                   int kernel_size);

  void draw_bounding_box();
  void draw_canonical_rectangle();
  void place_opengl_camera();

 private:
  Image depthmap;
  Image confidence;
  vcl_vector<vnl_double_4> planes;

  GLuint key_texture;
  vcl_vector<GLuint> ref_textures;

  // Some of these buffers are double because
  // when updating one buffer we read from the other.
  // Each buffer has its corresponding FramebufferObjet

  // a color buffer storing the best depth (RED) and the best scores (GREEN)
  GLuint best_depth_texture[2];
  FramebufferObject best_depth_fbo[2];
  int current_best_depth_texture;

  // a color buffer storing the sorted best scores found at current depth
  GLuint depth_score_texture[2];
  FramebufferObject depth_score_fbo[2];
  int current_depth_score_texture;

  // a color image storing the agregated score on its RED channel
  GLuint agregated_score_texture;
  FramebufferObject agregated_score_fbo;
  // a color image storing the warped image and potentially its matching score
  GLuint warped_image_texture;
  FramebufferObject warped_image_fbo;
  // a temporal texture
  GLuint tmp_texture;
  FramebufferObject tmp_fbo;

  // Shaders.
  ShaderProgram PlaneSweepShader;
  ShaderProgram WarpImageShader;
  ShaderProgram WarpImageSDShader;
  ShaderProgram WarpImageNCCShader;
  ShaderProgram AgregateScoreZNCCShader;
  ShaderProgram GrayScaleAndSquareShader;
  ShaderProgram PushInSortedListShader;
  ShaderProgram UpdateBestDepthShader;
  ShaderProgram ConvolutionShader;
  ShaderProgram ConvolutionRGBAShader;
};

#endif //__PlaneSweepStereoGPU_h__

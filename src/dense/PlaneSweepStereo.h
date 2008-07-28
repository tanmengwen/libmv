#ifndef __PlaneSweepStereo_h__
#define __PlaneSweepStereo_h__


#include "Camera.h"
#include "Image.h"
#include <vcl_vector.h>

class PlaneSweepStereo
{
public:
  typedef enum{SD, SSD, ZNCC} CorrelationType;

protected:
  Image key_image;
  Camera key_camera;
  vcl_vector<Image> ref_images;
  vcl_vector<Camera> ref_cameras;
  double bbox[2][3];

public:
  void add_reference_image( Image &im, Camera &c ) {
    ref_images.push_back(im);
    ref_cameras.push_back(c);
  }

  void set_key_image( Image &im, Camera &c) {
    key_image = im;
    key_camera = c;
  }

  void set_bounding_box(double xmin,double ymin,double zmin,
            double xmax,double ymax,double zmax) {
    bbox[0][0] = xmin;  bbox[1][0] = xmax;
    bbox[0][1] = ymin;  bbox[1][1] = ymax;
    bbox[0][2] = zmin;  bbox[1][2] = zmax;
  }

  int nrefs() {
    return ref_cameras.size();
  }

  virtual void compute(int nplanes, CorrelationType ct, int kernel_size) = 0;
  virtual Image get_depthmap() = 0;
  virtual Image get_confidence() = 0;

protected:
  int width() {
    return key_image.ni();
  }
  int height() {
    return key_image.nj();
  }

  void near_and_far_from_bounding_box() {
    near_and_far_from_bounding_box(key_camera);
    for(int i=0;i<nrefs();i++) {
      near_and_far_from_bounding_box(ref_cameras[i]);
    }
  }

  void near_and_far_from_bounding_box(Camera &c) {
    FrustumEstimator fs(c);
    for(int x=0;x<2;x++)
    for(int y=0;y<2;y++)
    for(int z=0;z<2;z++)
      fs.push(bbox[x][0],bbox[y][1],bbox[z][2]);

    c.near = fs.near;
    c.far = fs.far;
  }

  void get_plane_depths( vcl_vector<double> &depths, int ndepths ) {
    double n = key_camera.near;
    double f = key_camera.far;

    for(int i = 0; i <ndepths; ++i) {
      double alpha = (double(i) + .5)/double(ndepths);
      depths.push_back( n * f / (f + (n-f)*alpha) );
    }
  }
};

#endif //__PlaneSweepStereo_h__

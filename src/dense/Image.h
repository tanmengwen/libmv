#ifndef __Image_h__
#define __Image_h__


#include <vil/vil_image_view.h>

typedef vil_image_view<float> Image;

Image Image_load(const char *filename);
void Image_save(const Image &im, const char *filename);


#endif //__Image_h__

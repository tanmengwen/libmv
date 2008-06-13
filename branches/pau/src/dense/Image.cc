#include "Image.h"

#include <vil/vil_image_view.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>


Image Image_load(const char *filename)
{
	vil_image_view<vxl_byte> tmp;
	tmp = vil_load(filename);
	Image res(tmp.ni(),tmp.nj(),1,tmp.nplanes());
	vil_convert_cast(tmp,res);
	return res;
}
void Image_save(const Image &im, const char *filename)
{
	vil_image_view<vxl_byte> tmp(im.ni(),im.nj(),1,im.nplanes());
	vil_convert_cast(im,tmp);
	vil_save(tmp,filename);
}



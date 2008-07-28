#ifndef __mvdata_h__
#define __mvdata_h__

#include "Camera.h"
#include "Image.h"

#include <vcl_vector.h>


// a class for reading the multiview stereo evaluation data
class MiddleburyData {
public:
	int nimages;
	vcl_vector<Image> images;
	vcl_vector<vcl_string> image_paths;
	vcl_vector<Camera> cameras;
	const char *dir;
	const char *name;
	double bbox[2][3];

	MiddleburyData();
	static bool is_valid_data( const char *dir, const char *name );
	bool read( const char *_dir, const char *_name, bool load_images );
};

#endif //__mvdata_h__

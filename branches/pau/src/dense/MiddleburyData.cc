
#include "MiddleburyData.h"

#include "Camera.h"
#include "Image.h"

#include <vcl_vector.h>


MiddleburyData::MiddleburyData() {
	nimages=0;
	dir=name=NULL;
	bbox[0][0]=bbox[0][1]=bbox[0][2]=0;
	bbox[1][0]=bbox[1][1]=bbox[1][2]=0;
}

bool MiddleburyData::is_valid_data( const char *dir, const char *name ) {
	char buf[300];
	sprintf(buf, "%s/%s_par.txt", dir,name);
	FILE *tmp = fopen( buf, "rb" );
	if(tmp) {
		fclose(tmp);
		return true;
	}
	return false;
}

bool MiddleburyData::read( const char *_dir, const char *_name, bool load_images ) {
	dir=_dir;
	name=_name;

// open the dir/name_par.txt file
	char buf[300];
	sprintf(buf, "%s/%s_par.txt", dir,name);
	FILE *f = fopen( buf, "rb" );
	if(!f) return false;

// read the number of images
	int foo=0;
	foo = fscanf(f, "%i", &nimages );

	for( int i=0; i<nimages; i++ ) {
	// read the image
		char image_name[300];
		foo = fscanf( f, "%s", image_name );
		char image_path[300];
		sprintf(image_path, "%s/%s", dir,image_name );

		image_paths.push_back(vcl_string(image_path));

		Image im = Image_load(image_path);
		if(load_images)
			images.push_back( im );


	// read the camera parameters
		vnl_double_3x3 K;
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				foo = fscanf(f,"%lf",&K(i,j));
		vnl_double_3x3 R;
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				foo = fscanf(f,"%lf",&R(i,j));
		vnl_double_3 t;
		for(int i=0;i<3;i++)
			foo = fscanf(f,"%lf",&t(i));

		Camera c;
		c.set_KRt(K,R,t);
		c.width = im.ni();
		c.height = im.nj();
		cameras.push_back(c);
	}
	fclose(f);

        // read the bounding box
	sprintf(buf, "%s/%s_bbox.txt", dir,name);
	FILE *bbf = fopen( buf, "rb" );
	if(bbf) {
		for(int j=0;j<6;j++) foo = fscanf(bbf,"%lf",bbox[0]+j);
		fclose(bbf);
	}
	else return false;

	return true;
}


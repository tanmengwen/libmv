#include "Frame.h"


void Frame::compute_sift_features()
{
	char buf[1000];
	sprintf(buf, "sift %s --output=%s", image_filename.c_str(),sift_filename.c_str());

	int res = system(buf);
}


void Frame::load_sift_features()
{
	FILE *fin;
	fin = fopen(sift_filename.c_str(),"r");

	SiftFeature sf;
	int res = fscanf(fin, "%f %f %f %f", &sf.x, &sf.y, &sf.scale, &sf.angle);
	while(res==4) {
		for(int i=0;i<128;i++)
			fscanf(fin,"%d", sf.descriptor+i);
		features.push_back(sf);

		res = fscanf(fin, "%f %f %f %f", &sf.x, &sf.y, &sf.scale, &sf.angle);
	}
}


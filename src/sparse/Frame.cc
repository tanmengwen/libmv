#include "Frame.h"

void Frame::set_filenames(const std::string &ifn) {
	image_filename = ifn;
}




void Frame::compute_sift_features()
{
	int res;

	const char *tmp_pgm = "/tmp/libmv_frame.pgm";
	const char *tmp_key = "/tmp/libmv_frame.key";
	char buf[1000];
	sprintf(buf, "convert %s %s", image_filename.c_str(), tmp_pgm);
	res = system(buf);

	sprintf(buf, "sift %s --output=%s", tmp_pgm, tmp_key);
	res = system(buf);

	load_sift_features(tmp_key);

	sprintf(buf, "rm %s", tmp_pgm);
	res = system(buf);
	sprintf(buf, "rm %s", tmp_key);
	res = system(buf);
}


void Frame::load_sift_features(const char *sift_filename)
{
	FILE *fin;
	fin = fopen(sift_filename,"r");

	SiftFeature sf;
	int res = fscanf(fin, "%f %f %f %f", &sf.x, &sf.y, &sf.scale, &sf.angle);
	while(res==4) {
		for(int i=0;i<128;i++)
			fscanf(fin,"%d", sf.descriptor+i);
		features.push_back(sf);

		res = fscanf(fin, "%f %f %f %f", &sf.x, &sf.y, &sf.scale, &sf.angle);
	}
}


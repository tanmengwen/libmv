#ifndef __FRAME_H__
#define __FRAME_H__


#include <vector>
#include <string>
#include <iostream>
#include "SiftFeature.h"


class Frame
{
public:
	std::string image_filename;

	int width, height;
	std::vector<SiftFeature> features;

public:
	Frame() {}
	Frame(const std::string &ifn) {
		set_filenames(ifn);
	}
	void set_filenames(const std::string &ifn);

	void compute_sift_features();
	void load_sift_features(const char *sift_filename);
};


#endif //__FRAME_H__




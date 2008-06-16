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
	std::string sift_filename;  // sift features are computed by the
				    // ../extern/siftpp/sift executable and stored in this file

	int width, height;
	std::vector<SiftFeature> features;

public:
	Frame() {}
	Frame(const std::string &ifn) {
		set_filenames(ifn);
	}
	void set_filenames(const std::string &ifn, const std::string &sfn="null") {
		image_filename = ifn;

		if( sfn == std::string("null") ) {
			sift_filename = sift_filename_from_image_filename(ifn);
		} else {
			sift_filename = sfn;
		}
	}

	static std::string sift_filename_from_image_filename( const std::string &ifn ) {
		std::string res = ifn;

		int dot=res.size()-1;
		while(res[dot]!='.') dot--;

		res.replace(dot,res.size()-dot,".key");

		return res;
	}

	void compute_sift_features();
	void load_sift_features();
};


#endif //__FRAME_H__




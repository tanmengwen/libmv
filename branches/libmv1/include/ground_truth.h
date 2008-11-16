#ifndef _GROUND_TRUTH_H
#define _GROUND_TRUTH_H
#include "mv.h"
#include <ImfArray.h>
#include <json.h>

namespace mv {

// pure abstract
class GroundTruthLoader {
public:
	virtual int get_world_coord(int frame, double pixel_x, double pixel_y,
			double *X, double *Y, double *Z) = 0;
	virtual ~GroundTruthLoader() {};
};

class GroundTruthEXRLoader : public GroundTruthLoader {
public:
	GroundTruthEXRLoader() : currframe(-1) {};
	int load_ground_truth(const char *pattern, int first, int last);
	virtual int get_world_coord(int frame, double pixel_x, double pixel_y,
			double *X, double *Y, double *Z);
	virtual ~GroundTruthEXRLoader() {};
private:
	void load_frame(int frame);

	int first, last;
	int currframe;
	Imf::Array2D<float> rPixels;
	Imf::Array2D<float> gPixels;
	Imf::Array2D<float> bPixels;
	Imf::Array2D<float> aPixels;
	int width, height;
	const char* format;
};

struct V3 { double x[3]; V3() {x[0]=x[1]=x[2]=0;}   };
class GroundTruth {
public:
	GroundTruth () : robust(1) {};
	void set_loader(GroundTruthLoader* loader);
	void set_robust(int on) {robust = on;};
	void add_point(unsigned int frame, unsigned int track,
	               double px, double py);
	void set_estimated_track_position(unsigned int track,
	                                  double x, double y, double z);
	void estimated_track_position(unsigned int track,
								  double *x, double *y, double *z);
	mat residuals();
	double align();
	void export_truth_blender(const char* fn);
	unsigned int tracks();
	Json* tracks_to_json();
	Json* estimated_to_json();
private:
	GroundTruthLoader* loader;
	// Maps track # -> 3D point(s)
	std::map<int, std::vector<V3> > actual_positions;
	std::map<int, V3 > estimated_positions;
	int robust;
};

} // namespace mv

#endif // _GROUND_TRUTH_H

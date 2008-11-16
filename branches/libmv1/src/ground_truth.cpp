#include <ImfRgbaFile.h>
#include <ImfInputFile.h>
#include <ImathBox.h>
#include <ImfMatrixAttribute.h>
#include <json.h>

#include "mv.h"
#include "ground_truth.h"
#include "align.h"

namespace mv {

using namespace json;

unsigned int GroundTruth::tracks() {
	return actual_positions.size();
}

void GroundTruth::set_loader(GroundTruthLoader* loader) {
	this->loader = loader;
}

void GroundTruth::set_estimated_track_position(unsigned int track,
								  double x, double y, double z)
{
	V3 v;
	v.x[0] = x;
	v.x[1] = y;
	v.x[2] = z;

	assert (estimated_positions.find(track) == estimated_positions.end());
	estimated_positions[track] = v;
}

void GroundTruth::estimated_track_position(unsigned int track,
								  double *x, double *y, double *z)
{
	*x = estimated_positions[track].x[0];
	*y = estimated_positions[track].x[1];
	*z = estimated_positions[track].x[2];
}

void GroundTruth::add_point(unsigned int frame, unsigned int track,
			   double px, double py)
{
	assert(loader);
	V3 vert;
	int hit = loader->get_world_coord(frame, px, py,
			&(vert.x[0]), &(vert.x[1]), &(vert.x[2]));

	if (actual_positions.find(track) == actual_positions.end()) {
		std::vector<V3> verts;
		actual_positions[track] = verts;
	} 
//	printf("pushed back=(%f %f %f)\n", vert.x[0], vert.x[1], vert.x[2]);
	if (hit) {
		actual_positions[track].push_back(vert);
	} else {
		printf("missed structure; ignoring this point.");
	}
}

double GroundTruth::align()
{
	int total_points = 0;
	std::map<int, std::vector<V3> >::iterator it;
	for (it = actual_positions.begin(); it != actual_positions.end(); ++it) {
		total_points += it->second.size();
	}

	mat G(3,total_points);
	mat X(3,total_points);

	int i = 0;
	for (it  = actual_positions.begin();
	     it != actual_positions.end(); ++it) {
		assert (estimated_positions.find(it->first) != estimated_positions.end());
		const V3 &est = estimated_positions[it->first];
//		printf("estimated: %f %f %f\n", est.x[0],est.x[1],est.x[2]);
		std::vector<V3> &actuals = it->second;
		for (unsigned int j=0; j<actuals.size(); j++) {
			for (int k=0; k<3; k++) {
				G(k,i) = actuals[j].x[k];
				X(k,i) = est.x[k];
			}
			i++;
		}
	}
	assert(i == total_points);

	double error_thresh = 0;
	mat T;
	if (robust)
		robust_find_aligning_transform(G,X,&T,&error_thresh);
	else 
		find_aligning_transform(G,X,&T);

//	dpmat(G);
//	dpmat(X);
	// transform estimated positions accordingly
	std::map<int, V3>::iterator jt;
	for (jt = estimated_positions.begin();
	     jt != estimated_positions.end(); ++jt) {
		vec pt(4);
		pt[0] = jt->second.x[0];
		pt[1] = jt->second.x[1];
		pt[2] = jt->second.x[2];
		pt[3] = 1.0;
		pt = dot(T, pt);
		jt->second.x[0] = pt[0] / pt[3];
		jt->second.x[1] = pt[1] / pt[3];
		jt->second.x[2] = pt[2] / pt[3];
//		dpmat(pt);
	}
	return error_thresh;
}

using namespace Imf;
using namespace Imath;

static void
readRGBA (const char fileName[],
		Array2D<float> &rPixels,
		Array2D<float> &gPixels,
		Array2D<float> &bPixels,
		Array2D<float> &aPixels,
		int &width, int &height)
{
	InputFile file (fileName);
	Box2i dw = file.header().dataWindow();
	width = dw.max.x - dw.min.x + 1;
	height = dw.max.y - dw.min.y + 1;
	rPixels.resizeErase (height, width);
	gPixels.resizeErase (height, width);
	bPixels.resizeErase (height, width);
	aPixels.resizeErase (height, width);
	FrameBuffer frameBuffer;
	frameBuffer.insert ("R", // name
			Slice (FLOAT, // type
				(char *) (&rPixels[0][0] - // base
					dw.min.x -
					dw.min.y * width),
				sizeof (rPixels[0][0]) * 1, // xStride
				sizeof (rPixels[0][0]) * width,// yStride
				1, 1, // x/y sampling
				3.0)); // fillValue
	frameBuffer.insert ("G", // name
			Slice (FLOAT, // type
				(char *) (&gPixels[0][0] - // base
					dw.min.x -
					dw.min.y * width),
				sizeof (gPixels[0][0]) * 1, // xStride
				sizeof (gPixels[0][0]) * width,// yStride
				1, 1, // x/y sampling
				4.0)); // fillValue
	frameBuffer.insert ("B", // name
			Slice (FLOAT, // type
				(char *) (&bPixels[0][0] - // base
					dw.min.x -
					dw.min.y * width),
				sizeof (bPixels[0][0]) * 1, // xStride
				sizeof (bPixels[0][0]) * width,// yStride
				1, 1, // x/y sampling
				5.0)); // fillValue
	frameBuffer.insert ("A", // name
			Slice (FLOAT, // type
				(char *) (&aPixels[0][0] - // base
					dw.min.x -
					dw.min.y * width),
				sizeof (aPixels[0][0]) * 1, // xStride
				sizeof (aPixels[0][0]) * width,// yStride
				1, 1, // x/y sampling
				6.0)); // fillValue
	file.setFrameBuffer (frameBuffer);
	file.readPixels (dw.min.y, dw.max.y);

	// read camera ttrib
	const M44fAttribute *cameraOrientationMat =
	file.header().findTypedAttribute <M44fAttribute> ("cameraOrientationMat");
	if (cameraOrientationMat)
		std::cerr << "cameraOrientationMat\n" << cameraOrientationMat->value() << std::flush;
	const M44fAttribute *cameraProjectionMat =
	file.header().findTypedAttribute <M44fAttribute> ("cameraProjectionMat");
	if (cameraProjectionMat)
		std::cerr << "cameraProjectionMat\n" << cameraProjectionMat->value() << std::flush;
}

void GroundTruthEXRLoader::load_frame(int frame)
{
	char fnamein[200];
	if (this->currframe == frame)
		return;
	currframe = frame;
	sprintf(fnamein, format, currframe);
	fprintf(stderr, "# Reading EXR file %s\n", fnamein);
	readRGBA(fnamein, rPixels, gPixels, bPixels, aPixels, width, height);
	fprintf(stderr, "# Read EXR file %s; (%d,%d)\n", fnamein, width,height);

}

int  GroundTruthEXRLoader::load_ground_truth(const char *pattern, int first, int last)
{
	format = pattern;
	this->first = first;
	this->last = last;

	return 1; // FIXME
}

int GroundTruthEXRLoader::get_world_coord(int frame, double pixel_x, double pixel_y,
			double *X, double *Y, double *Z) {

//	printf("EXRLOADER: frame=%d, currframe=%d\n",frame,currframe);
	if (frame != currframe)
		load_frame(frame);

	// FIXME check off-by-one pixel conventions (where is 0,0?)
	int x = (int)nearbyint(pixel_x);
	int y = (int)nearbyint(pixel_y);

	assert (0 < x);
	assert (x < width);
	assert (0 < y);
	assert (y < height);

	// FIXME check for W
	*X = rPixels[y][x];
	*Y = gPixels[y][x];
	*Z = bPixels[y][x];
	if(aPixels[y][x] == 0.0) 
		fprintf(stderr, "hit infinity x=%d y=%d frame=%d\n",x,y,frame);

	return (int)aPixels[y][x];
}

mat GroundTruth::residuals()
{
	int total_points = 0;
	std::map<int, std::vector<V3> >::iterator it;
	for (it = actual_positions.begin(); it != actual_positions.end(); ++it) {
		total_points += it->second.size();
	}

	mat resids(3,total_points);

	int i = 0;
	for (it  = actual_positions.begin();
	     it != actual_positions.end(); ++it) {
		V3 est = estimated_positions[it->first];
		std::vector<V3> &actuals = it->second;
		for (unsigned int j=0; j<actuals.size(); j++) {
			for (int k=0; k<3; k++) {
				resids(k,i) = actuals[j].x[k] - est.x[k];
			}
			i++;
		}
	}
	return resids;
}

void GroundTruth::export_truth_blender(const char* fn_base)
{
	EuclideanReconstruction etruth;
	std::map<int, std::vector<V3> >::iterator it;
	for (it = actual_positions.begin(); it != actual_positions.end(); ++it) {
		std::vector<V3> &actuals = it->second;
		for (unsigned int j=0; j<actuals.size(); j++) {
			EuclideanPointStructure *eps = new EuclideanPointStructure();
			eps->X[0] = actuals[j].x[0];
			eps->X[1] = actuals[j].x[1];
			eps->X[2] = actuals[j].x[2];
			etruth.push_structure(eps);
		}
	}

	char fn[300];
	sprintf(fn, "%s_truth.py", fn_base);
	etruth.write_blender(fn);

	EuclideanReconstruction eest;
	std::map<int, V3>::iterator jt;
	for (jt = estimated_positions.begin();
	     jt != estimated_positions.end(); ++jt) {
		EuclideanPointStructure *eps = new EuclideanPointStructure();
		eps->X[0] = jt->second.x[0];
		eps->X[1] = jt->second.x[1];
		eps->X[2] = jt->second.x[2];
		eest.push_structure(eps);
	}
	sprintf(fn, "%s_est.py", fn_base);
	eest.write_blender(fn);
}

Json* GroundTruth::tracks_to_json() {
	Json *ret = NewArray();
	std::map<int, std::vector<V3> >::iterator it;
	for (it = actual_positions.begin(); it != actual_positions.end(); ++it) {
		std::vector<V3> &actuals = it->second;
		Json &arr = ret->append_new_array();
		for (size_t j=0; j<actuals.size(); j++) {
			Json &actual_point = arr.append_new_array();
			actual_point.append(actuals[j].x[0]);
			actual_point.append(actuals[j].x[1]);
			actual_point.append(actuals[j].x[2]);
		}
	}
	return ret;
}

Json* GroundTruth::estimated_to_json() {
	Json *ret = NewArray();
	std::map<int, V3>::iterator it;
	for (it = estimated_positions.begin();
			it != estimated_positions.end(); ++it) {
		Json &point = ret->append_new_array();
		point.append(it->second.x[0]);
		point.append(it->second.x[1]);
		point.append(it->second.x[2]);
	}
	return ret;
}

} // namespace mv


#ifndef _MV_H
#define _MV_H

#include <vector>
#include <map>
#include <set>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/banded.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/foreach.hpp>

// Useful defines
#define foreach BOOST_FOREACH

#ifdef TDEBUG
#define dprintf(...) { printf("%s:%d: ",__FUNCTION__, __LINE__); printf( __VA_ARGS__); }
#define dprintmat(matrix) {printf(#matrix"="); printnumpymatrix(matrix);}
#define dprintmat2(name, matrix) {printf(name"="); printnumpymatrix(matrix);}
#else
#define dprintf(...)
#define dprintmat(matrix)
#define dprintmat2(name, matrix)
#endif

#define TDEBUG
#ifdef TDEBUG
//#define dpmat(matrix) {std::cout << #matrix"=\t" << matrix << std::endl;}
#define dpmat(matrix) {std::cout << #matrix" = "; pnumpy(matrix);}
#define dpmatt(matrix) {ptst(matrix, #matrix);}
#else
#define dpmat(matrix)
#endif

#include "config.h"

extern "C" {
#include "klt.h"
}

namespace json {
	class Json;
}

namespace mv {

using std::string;
using json::Json;

// I don't like typing
namespace ublas = boost::numeric::ublas;
using ublas::range;
using ublas::prod;

typedef ublas::matrix<double> mat;
typedef ublas::vector<double> vec;

typedef ublas::matrix<double, ublas::column_major> fmat;
typedef ublas::banded_matrix<double> bmat;

typedef ublas::matrix_range<mat> submat;
typedef ublas::vector_range<vec> subvec;
typedef ublas::matrix_vector_range<mat> submatvec;
typedef ublas::identity_matrix<double> eye;
typedef ublas::zero_matrix<double> mzeros;
typedef ublas::zero_vector<double> vzeros;

// Help ublas become more Numpy-like
void fill(vec &v, double val);
void fill(vec &v, double *val);
void fill(mat &m, double val);
void fill(fmat &m, double val);
void fill(mat &m, double *val);
void zero(vec &v);
void zero(mat &m);
vec operator*(const vec& a, const vec& b);
mat operator*(const mat& a, const vec& b);
mat operator*(const vec& b, const mat& M);
vec operator-(const vec& a, double b);
double amax(const vec& a);
double amin(const vec& a);
vec aexp(const vec& a);
vec sum1(const mat &M);
vec sum2(const mat &M);
mat transpose(const mat &M);
vec ravel(const mat &M);
mat reshape(const vec &v, size_t rows, size_t cols);
double norm(mat const &m); // frobenius norm
double det3(const mat &a);
#define dot prod

// For parameterizing homogenous quantities
vec sphere_to_disk(const vec &P);
vec disk_to_sphere(const vec &R);
mat jacobian_sphere_disk(const vec &X);

// Debugging
void pnumpy(const vec &a);
void pnumpy(const mat &M);

//======================================================================
//======================================================================
//======================================================================

class Feature;
class Track;

class Frame : public std::vector<Feature*>
{
public:
	Frame() : orig_frame_ind(-1){};
	Frame(int n)
		{ this->resize(n); };
	void push_feature(Feature* feat);
	void push_feature(Track* track, Feature* feat);
	void set_frame_number(int n) {frame_num = n;}
	int get_frame_number() {return frame_num;}
	string get_filename() {return filename;}
	void set_filename(string fn) {filename = fn;}
	void set_orig_ind(int n) {orig_frame_ind = n;}
	int get_orig_ind() {return orig_frame_ind;}

private:
	int orig_frame_ind;
	int frame_num;
	string filename;
};

// A bag of frames
class FrameCollection : public std::vector<Frame*>
{
public:
	FrameCollection() : std::vector<Frame*>() {};
	FrameCollection(int n)
		{ this->resize(n); };
private:

};

class PointFeature;
class UnscentedTransform;
class TrackedSequence : public FrameCollection
{
public:
	TrackedSequence() : FrameCollection()
		{debug=false; started=false;}
	TrackedSequence(size_t n) : FrameCollection(n)
		{debug=false; started=false;}
	void start_tracking(int cols, int rows);
	void stop_tracking();
	void set_size(int rows, int cols)
		{nrows = rows; ncols = cols;}
	int get_width() const
		{return ncols;}
	int get_height() const
		{return nrows;}
	void set_height(int height) {
		nrows = height;
	}
	void set_width(int width) {
		ncols = width;
	}

	void push_frame(unsigned char *image, string fname, int orig_frame_number);
	Frame* add_frame();
	void enable_debug_output() { debug = true; }
	void disable_debug_output() { debug = false; }
	/**
	 * Number the features and tracks.
	 */
	void number_items();

	size_t get_ntracks() {return tracks.size();};
	void track_pgm_sequence(const char* format, int start, int end);
	void drop_track(Track *track);
	void set_format(const string &fmt);
	const string &get_format() const;

	string dump_cpp_ascii();
	Json* dump_json();
	Feature* add_point_feature(unsigned int frame, unsigned int track, double x, double y);
	void set_config(const Config &config);
	void track_and_replace_features(unsigned char *image);
	bool track_sigma_points_and_compute_sigma(
		PointFeature *pf_previous,
		PointFeature *pf,
		UnscentedTransform *ut);

	// cringe
	std::vector<Track*> tracks;
private:
	unsigned char* last;
	string format_;
	int nrows, ncols;
	KLT_TrackingContext tc;
	KLT_FeatureList fl;
	int nfeatures;
	bool debug, started;
	std::vector<Track*> recent_tracks;
	Config config_;
};

#include "feature.h"

enum TrackType { POINT=10, LINE=40 };

// A track of a feature 
class Track : public std::vector<Feature*>
{
public:
	TrackType get_type()
		{ return tracktype; };
	void set_type(TrackType type)
		{ tracktype = type; };
	void push_feature(Feature* feat) {
		this->push_back(feat);
	}
	size_t get_number()
		{ return number; }
	void set_number(size_t number)
		{ this->number = number; }
private: 
	TrackType tracktype;
	size_t number;
};

class PointTrack : public Track
{
public:
	PointTrack() { set_type(POINT); };
};

// Generally this links 1-1 with a track
class PointStructure;
class EuclideanPointStructure;
class Structure
{
public:
	Structure() : number(0), track(NULL) {} ;
	void set_track(Track* track)
		{ this->track = track; }
	Track *get_track()
		{ return this->track; }
	void set_type(TrackType type)
		{ this->type = type; };
	TrackType get_type()
		{ return type; }
	size_t get_track_number()
		{ return track->get_number(); }
	size_t get_number()
		{ return number; }
	void set_number(size_t n)
		{ number = n; }
	void copy_into(Structure* s) {
		s->set_type(type);
		s->set_track(track);
		s->set_number(number);
	}
	virtual Json* dump_json() = 0;
	virtual Structure* copy() = 0;
	virtual ~Structure() {};

	// double dispatch comparison
	virtual bool base_equals(const Structure &) const;
	virtual bool equals(const Structure &) const = 0;
	virtual bool equals(const PointStructure &) const;
	virtual bool equals(const EuclideanPointStructure &) const;

	size_t number;
private:
	TrackType type;
	Track* track;
};

class PointStructure : public Structure, public vec
{
public:
	PointStructure() : vec(4)
		{ init(); }
	PointStructure(Track *track) : vec(4) { 
		init();
		set_track(track);
	}
	void init() {
		zero(*this);
		set_type(POINT);
		set_track(NULL);
	}
	Structure* copy();
	virtual Json* dump_json();

	virtual bool equals(const Structure &) const;
	virtual bool equals(const PointStructure &) const;
	virtual bool equals(const EuclideanPointStructure &) const;


	// Estimated sigma2 for residuals (of rayleigh distribution)
	double sigma2;
};

class ProjectiveCamera;
class EuclideanCamera;
class Camera
{
public:
//	virtual Feature* project(Structure &X) = 0;
	Camera() : frame(NULL), number(99993) {};
	virtual ~Camera(){};
	void set_frame(Frame* f)
		{ this->frame = f; };
	size_t get_frame_number()
		{ return this->frame->get_frame_number(); }
	Frame* get_frame()
		{ return this->frame; }
	size_t get_number()
		{ return this->number; }
	void set_number(size_t n)
		{ this->number = n; }
	void copy_into(Camera* c)
		{ c->set_frame(frame); c->set_number(number); }

	virtual Camera* copy() = 0;
//	virtual bool can_project(Structure* s) = 0;
//	virtual double project(Structure* s, Feature *f) = 0;
	virtual double reprojection_error(Structure *s, Feature *f) = 0;

	virtual Json* dump_json() const = 0;

	// double dispatch comparison
	virtual bool base_equals(const Camera&) const;
	virtual bool equals(const Camera&) const = 0;
	virtual bool equals(const ProjectiveCamera&) const;
	virtual bool equals(const EuclideanCamera&) const;
	
	Frame* frame;
	size_t number;
};

class ProjectiveCamera : public Camera, public mat
{
public:
	ProjectiveCamera() : mat(3,4) {zero(*this);};
	ProjectiveCamera(Frame* frame) : mat(3,4)
		{set_frame(frame);};
	ProjectiveCamera(Frame* frame, mat &P) : mat(3,4)
		{set_frame(frame); this->assign(P);};
	PointFeature project_point(PointStructure *X);
	double reprojection_error(Structure *s, Feature *f);
	virtual bool can_project(Structure *s)
		{ return (s->get_type() == POINT); }
	virtual Json* dump_json() const;
	virtual Camera* copy();
	virtual bool equals(const Camera&) const;
	virtual bool equals(const ProjectiveCamera&) const;
//	virtual ~ProjectiveCamera() {};
};


class EuclideanPointStructure : public Structure
{
public:
	EuclideanPointStructure(): X(3)
		{ init(); }
	void init() {
		zero(X);
		set_type(POINT);
		set_track(NULL);
	}
	Structure* copy() {
		// FIXME
		assert(0);
	}
	virtual bool equals(const Structure &) const;
	virtual bool equals(const PointStructure &) const;
	virtual bool equals(const EuclideanPointStructure &) const;

	vec X;
	virtual Json* dump_json();
};

class EuclideanCamera : public Camera
{
public:
	EuclideanCamera() : K(3,3),R(3,3),t(3) {
		zero(K);
		zero(R);
		zero(t);
	};
	Camera* copy() {assert(0); return NULL;}
	double reprojection_error(Structure *s, Feature *f)
	{ 
		EuclideanPointStructure *ps = (EuclideanPointStructure*)s;
		PointFeature *pf = (PointFeature*)f;
		vec tmp = dot(R, ps->X) + t;
		vec est = dot(K, tmp);
		est /= est[2];

		double dx = pf->x() - est[0];
		double dy = pf->y() - est[1];
		double e2 = dx*dx+dy*dy;
		return e2;
	}
	PointFeature project_point(EuclideanPointStructure *X) {
		(void)X; return PointFeature();
	}
	virtual Json* dump_json() const;
	virtual bool equals(const Camera&) const;
	virtual bool equals(const EuclideanCamera&) const;
	mat K;
	mat R;
	vec t;
};
typedef std::pair<Camera*,Structure*> CToS;
typedef std::pair<Structure*,Camera*> SToC;

/**
 * Store a reconstruction.
 */
class Reconstruction
{
	friend class MeasurementVisitor;
public:
	Reconstruction(): ts(NULL) {}
	virtual void trim_outliers() {};

	void push_camera(Camera *s) {
		cameras.push_back(s);
	}

	void push_structure(Structure *s) {
		structure.push_back(s);
	}

//	void push_measurement(MeasurementVisitor &mv) {
//		push_measurement(mv.camera(), mv.structure(), mv.measurement());
//	}

	void push_measurement(Camera *cam, Structure *s, Feature *feat) {
		assert(cam != NULL);
		assert(s != NULL);
		assert(feat != NULL);
		measurements[CToS(cam, s)] = feat;
		cam_sees.insert(CToS(cam, s));
		visible_in.insert(SToC(s, cam));
	}

	void camera_sees_all(Camera* cam) {
		foreach (Structure *s, structure) {
			cam_sees.insert(CToS(cam, s));
			visible_in.insert(SToC(s, cam));
		}
	}

	/**
	 * Duplicate cameras/etc into this reconstruction. The underlying
	 * camera and structure objects are duplicated, but the references to
	 * tracks still point outside the structure.
	 */
	Reconstruction *copy_into(Reconstruction* r);

	/**
	 * Duplicate this reconstruction. Does not duplicate underlying tracks
	 * or features; only cameras, structure, and visibility.
	 */
	virtual Reconstruction *copy() = 0;

	/**
	 * Find cameras common between two reconstructions by checking the
	 * underlying frame number.
	 */
	void common_cameras(Reconstruction &other,
			std::vector<size_t> &common_this,
			std::vector<size_t> &common_other);


	/**
	 * Merge another reconstruction into this one
	 */
	void merge(Reconstruction &other);

	/**
	 * Calculate RMS reprojection error over all features
	 */
	double rms_reprojection_error();
	double mean_reprojection_error();
	void number_items();
	void print_residuals();
	void drop_structure(std::set<Structure*> &str);

	TrackedSequence* get_tracked_sequence() {
		return ts;
	}
	void set_tracked_sequence(TrackedSequence* ts) {
		this->ts = ts;
	};
	virtual Json* dump_json();

	virtual ~Reconstruction() {};

	// Deletes contents of the reconstruction
	virtual void delete_contents() {
		for (size_t i=0; i<cameras.size(); i++)
			delete cameras[i];
		for (size_t i=0; i<structure.size(); i++)
			delete structure[i];
		std::map<CToS,Feature*>::iterator it;
		for (it=measurements.begin(); it!=measurements.end(); ++it)
			delete it->second;
		measurements.clear();
		cam_sees.clear();
		visible_in.clear();
	}

	virtual void bundle_adjust() = 0;

	Feature *get_feat(size_t cam_num, size_t str_num) {
		return measurements[CToS(cameras[cam_num], structure[str_num])];
	}

	std::vector<Camera*> cameras;
	std::vector<Structure*> structure;
	std::multimap<Camera*, Structure*> cam_sees;
	std::map<CToS, Feature*> measurements;
	std::multimap<Structure*, Camera*> visible_in;
	TrackedSequence *ts;
private:
};

class ProjectiveReconstruction: public Reconstruction
{
public:
	// bundle adjust etc
	virtual void bundle_adjust();

	/* Uses the algorithm from "Automatic Camera Recovery for Closed or Open Image
	 * Sequences" number II. */
	mat align_oneview(ProjectiveReconstruction &other);
	bool robust_oneview_align(ProjectiveReconstruction &pr);

	/**
	 * Fully reconstruction a tracked sequence
	 */
	void reconstruct(TrackedSequence &ts, size_t ransac_rounds=100);

	/**
	 * Normalize cameras such that the first camera is [I|0]
	 */
	void normalize_camera(int num=0);

	/**
	 * Print out numpy representation of the camera matricies
	 */
	void print_cameras();
	void print_structure();
	virtual void trim_outliers();

	ProjectiveCamera* pcam(size_t num)
		{return (ProjectiveCamera*) cameras[num];}
	PointStructure* pstr(size_t num)
		{return (PointStructure*) structure[num];}

	/**
	 * Transform points according to H and cameras according to H^-1:
	 *
	 * :   X' = HX
	 * :   P' = PH^-1 
	 *
	 * Parameters:
	 *     H - 4x4 homography
	 */
	void transform(mat &H);

	virtual Json* dump_json();

	Reconstruction *copy();
	void set_config(const Config &conf) {
		config_ = conf;
	}

	/**
	 * Estimate an approximation of the covariance of the reconstructed
	 * scene points.
	 */
	void estimate_errors();

	// Estimated sigma2 for residuals (of rayleigh distribution)
	double sigma2;

	bool   use_huber_cost_;
	double huber_threshold_;
	bool   use_mahalanobis_distance_;
	Config config_;
};

class EuclideanReconstruction: public Reconstruction
{
public:
	void set_config(const Config &conf) {
		config_ = conf;
	}
	EuclideanCamera *ecam(size_t i) {return (EuclideanCamera*)cameras[i];}
	void print_cameras();
	void print_structure();
	virtual void bundle_adjust();
	void write_blender(const char* fn);
	Reconstruction* copy() {assert(0); return NULL;} // FIXME
	virtual Json* dump_json();
	bool   use_huber_cost_;
	double huber_threshold_;
	bool   use_mahalanobis_distance_;
	Config config_;
};

//typedef PointFeature* ThreeFrameTrack[3];
struct ThreeFrameTrack {
	PointFeature *tfs[3];
	PointFeature *&operator[](size_t i) 
		{return tfs[i];}
};

/**
 * Visitor class to simplify iterating over all correspondences.
 */
class MeasurementVisitor
{
public:
	MeasurementVisitor(Reconstruction *over) {
		this->over = over;
		this->str_idx = 0;
		this->str_idx_end = over->structure.size();
		reset_range();
	}
	MeasurementVisitor(Reconstruction *over, size_t str_idx) {
		this->over = over;
		this->str_idx = str_idx;
		this->str_idx_end = str_idx+1;
		reset_range();
	}
	void reset_range() {
		assert(str_idx < str_idx_end <= over->structure.size());
		// FIXME FIXME it *IS* possible to have no structure
		if (over->structure.size() == 0)
			return;
		Structure* ss = over->structure[str_idx];
		point_range = over->visible_in.equal_range(ss);
		p = point_range.first;
//		assert(point_range.first != point_range.second);
	};
	bool next() {
		if (over->structure.size() == 0)
			return false;
		if (p == point_range.second) {
			str_idx++;
			if (str_idx >= str_idx_end)
				return false;
			reset_range();
		}

		s = over->structure[str_idx];
		c = p->second;
		CToS ctos(c, s);
		f = over->measurements[ctos];

		++p;

		return true;
	};
	Camera* camera()
		{ return c; }
	Structure* structure()
		{ return s; }
	Feature* measurement()
		{ return f; }
private:
	size_t str_idx;
	size_t str_idx_end;
	Feature* f;
	Camera* c;
	Structure* s;
	std::multimap<Structure*, Camera*>::iterator p;
	std::pair<std::multimap<Structure*, Camera*>::iterator,
		  std::multimap<Structure*, Camera*>::iterator >
			  point_range;
	Reconstruction *over;
};

//======================================================================
//================= Linear Algebra =====================================
//======================================================================

double dot(const vec &a, const vec &b);
mat crossmat(vec &v);
double nullspace(const mat &M, vec &v);
double nullspace(const fmat &M, vec &v);
double nullspace(const mat &M, vec &v1, vec &v2);
double nullspace(const fmat &M, vec &v1, vec &v2);
bmat svd(mat &M, fmat &U, fmat &Vt);
bmat svd(fmat &M, fmat &U, fmat &Vt);
mat inv(const mat &M);
mat pinv(const mat &M);
void inv2inplace(mat *M);
void sqrtm(mat &M, mat *res);

/**
 * Decompose A3x3 as A = RQ, such that R is upper triangular and Q is a
 * rotation matrix. If det(A) > 0, then the transformation Q will also have
 * negative determinant to force all(diag(R)>0).
 */
void rq(const mat &M, mat &R, mat &Q);
int rpoly(double *op, int degree, double *zeror, double *zeroi);
int gsl_poly_solve_cubic (double a, double b, double c,
                          double *x0, double *x1, double *x2);


//======================================================================
//================= Multiview Estimation ===============================
//======================================================================

void sba_proj_projective(int jj, int ii, double *aj, double *bi, double *xij, void *adata);
mat normalizing_homography(mat points);


/**
 * Compute a 1-parameter family of cameras from 5 correspondences
 *
 * Finds the camera pencil for five points assuming the world coordinates are
 * E1...E5 as specified above. This is a slightly smarter version than the
 * stardard DLT, because we know the world coordinates form a martix looking
 * like [I|1s], i.e.:
 *
 * :     [x1 x2 x3 x4 x5] = P*[1 0 0 0 1]
 * :                          [0 1 0 0 1]
 * :                          [0 0 1 0 1]
 * :                          [0 0 0 1 1]
 *
 * where xn are 3x1 homogenous vectors representing image points. This implies
 *
 * :    P = [a1*x1 a2*x2 a3*x3 a4*x4]
 *
 * where a1...a4 are scalars, with the condition that
 *   
 * :    -a5*x5 = a1*x1 + a2*x2 + a3*x3 + a4*x4
 *
 * In other words, to recover P it is sufficient to find the a's. Rearranging:
 *
 * :                                     T
 * :    [x1 x2 x3 x4 x5]*[a1 a2 a3 a4 a5] = 0 (3x1)
 *      
 * which is readily solved for a1..a5 by the SVD. Because these are homogenous
 * quantities, we ignore a5 and set P as above. For numeric stability, the
 * points are first normalized (note that this is _not_ a projective distortion
 * and preserves the noise model, unlike the other method of finding the camera
 * pencils which involves projectively transforming the image coordinates in
 * image to a standard basis) before solving for the cameras, then corrected
 * for the normalization after.
 *
 * Parameters:
 *     A - Destination matrix for first camera
 *     B - Destination matrix for second camera
 *
 * On Return:
 *     A and B are projection matricies such that P = uA + vB and xn = P*En
 */
void five_point_camera_pencil(const mat &points, mat &A, mat &B);


/**
 * Compute projective structure and motion from 6-points in 3-views
 *
 * x1...x6 refer to (homogenous) image coordinates in one of the three views
 * X1...X6 refer to (homogenous) world coordinates (in R^4)
 * P is 3x4 camera matrix. In a euclidean frame P = K[R|t] but in general is
 * any 3x4 matrix. P is also a homogenous quantity and is only defined up to a
 * scale factor.
 *
 * 1. Compute the three pencils of cameras _exactly_ projecting the first 5
 *    points into the three views. This gives As[0..2] and Bs[0..2].
 *
 * 2. Form Q (4x4) = A^T [x6]x B - B^T [x6]x A (equation 5 in the paper)
 *    for each of the cameras 0..2. We need this because X6 must live on the
 *    quadric Q
 *
 * 3. From Q[0..2] find w^i as 
 *    w^i = (Q12 Q13 Q23 Q24 Q34)
 *    for each camera i (i.e. from each Q[i])
 *
 * 4. Form W = stack of w^i for i=1..3 (i.e. W is a 3x5 matrix)
 *    Use the SVD to find a basis for the 2-dimensional nullspace.
 *    Be worried if there is one or more singular values (besides the two
 *    zeros) which are very small.
 *
 * 5. Solve the homogenous equation Z = alpha Z1 + beta Z2 where Z1 and Z2 are
 *    the basis for the nulspace of W, based on the cubic constraint (10) This
 *    part requires finding the cubic roots of a homogenous equation (i.e.
 *    only the ratio of alpha and beta matters). The only difficulty here is
 *    what to do when alpha or beta is zero. This can be detected because
 *    either all the coefficients except one will be zero. Otherwise
 *    arbitrarily picking alpha=1 or beta=1 is fine. Then take
 *    Z = (a b c d e)
 *
 * 6. Since there may be 3 solutions to the cubic, repeat the next steps for
 *    Z = alpha[0..2] Z1 + Z2. Choose the one with the smallest reprojection
 *    error on the 6th point.
 *
 * 7. Solve for X6 as the null vector of the design matrix (9)
 *    [e-d   0   0   a-b]
 *    [e-c   0   a    0 ]
 *    [d-c   b   0    0 ]
 *    [ 0   e-b a-d   0 ]
 *    [ 0    e   0   a-c]
 *    [ 0    0   d   b-c]
 *
 * 8. Calculate (u,v) via the SVD as the solution to
 *     
 *      ([x6]x * [AX6|BX6]) [u] = 0
 *                          [v]
 *    for each camera.
 *
 *    This chooses the particular camera among the pencil of cameras for this
 *    image, which minimizes algebraic error.
 *
 * 9. Among the choices of alpha, choose the one with the smallest geometry
 *    error.
 *
 * 10. Optionally do bundle adjustment.
 *
 * For more detail see the original paper:
 *
 * [1] A Six Point Solution for Structure and Motion (ECCV 2000).
 *     F. Schaffalitzky, A. Zisserman, R. I. Hartley, P. H. S. Torr 
 *     http://citeseer.ist.psu.edu/295420.html
 *
 * Extensive information on multiple view geometry is found in the truly
 * excellent book "Multiple View Geometry" by Richard Hartley and Andrew
 * Zisserman, 2nd edition. I refer to it as HZ in comments.
 *
 * Contact Keir if you have other questions: my last name at gmail
 */
void six_point_three_view(std::vector<ThreeFrameTrack> &tft,
		std::vector<ProjectiveReconstruction> &reconstructions);


/**
 * Optimally triangulate a point from two views.
 *
 * Given noisy x1 and x2 measured points in images 1 and 2, and a fundamental
 * matrix F, compute the closest points x1' and x2' that exactly obey the
 * fundamental constraint 
 *
 * :       T
 * :    x1' * F * x2' = 0
 *
 * where 'closest' is in the L2 norm sense (i.e. squared distance).
 *
 * Parameters:
 *      x1 - Point in the first image 
 *      x2 - Point in the second image
 *       F - Fundamental matrix (3x3)
 *     x1p - Optimally triangulated point in the first image
 *     x2p - Optimally triangulated point in the second image
 *
 * See Also:
 *     Pages 317-318 of Hartley & Zisserman
 */
void find_optimal_points_given_fundamental(
		const vec &x1, const vec &x2,
		const mat &F,
		vec &x1p, vec &x2p);

/**
 * Triangulate a point in 3D from two views via the DLT
 *
 * Calculates the 3D location of a point from its image in two views. This
 * method minimzes the algebraic error rather than any meaningful geometric
 * error, so be sure to precondition the points first. For example, use
 * <find_optimal_points_given_fundamental>, to find the two closest points that
 * exactly obey epipolar geometry, then use this function for accurate
 * triangulation.
 *
 * The 3rd componet of x1 and x2 are ignored.
 *
 * Parameters:
 *      P1 - First camera
 *      P2 - Second camera
 *      x1 - Point in the first image 
 *      x2 - Point in the second image
 * 
 * Returns:
 *       A R4 homogenous vector such that
 *
 *           x1 = P1*X and 
 *           x2 = P2*X
 *
 * See Also:
 *     Page 312 of of Hartley & Zisserman
 */
vec triangulate_dlt(const mat &P1, const mat &P2, const vec x1, const vec x2);

/**
 * Find epipoles of a fundamental matrix
 *
 * Find the epipoles of a fundamental matrix such that ex^2 + ey^2 == 1. The
 * epipoles are computed from the left and right nullspaces of the F matrix.
 *
 * Parameters:
 *     F - Fundamental matrix (3x3)
 *    e1 - Epipole in image 1
 *    e2 - Epipole in image 2
 */
void find_epipoles_from_fundamental(const mat &F, vec &e1, vec &e2);


/**
 * Calculate the fundamental matrix from two general camera matricies.
 *
 * Given two camera matricies, calculate the fundamental matrix between them
 * via the equation:
 *
 * :                  +
 * :   F = [e2] P2 * P
 * :           x
 * 
 * where e2 = P2*C, and P*C = 0 (i.e. C is the camera center of camera 1)
 *
 * Parameters:
 *    P1 - First camera matrix
 *    P2 - Second camera matrix
 *     F - Output fundamental matrix
 *
 * See Also:
 *     Page 246 of Hartley & Zisserman.
 */
void find_fundamental_from_camera_matricies(
		const mat &P1, const mat &P2, mat &F);

/**
 * Calculate a projective reconstruction by Sturm-Triggs factorization
 *
 * Given a set of points in a set of images, such that each point is visible in
 * each frame, compute the projective structure via the factorization
 * algorithm. The algorithm assumes all the projective depths are 1 to start
 * the process. Note that this algorithm is very sensitive to outliers, and
 * should not be used until after outliers are eliminated.
 *
 * Parameters:
 *    points - The points in each view, stored as points[view][number]
 *        pr - The calculated projective reconstruction
 *
 * See Also:
 *     Page 445 of Hartley & Zisserman.
 */
void factorize_projective_reconstruction(
		const std::vector<std::vector<PointFeature*> > points, 
		const std::vector<Frame*> frames, 
		ProjectiveReconstruction &pr);

/**
 * Calculate a homography to align projective reconstruction A to B.
 *
 * This is designed for use as a RANSAC kernel
 *
 * Parameters:
 *   PBplusPA - The matrix pinv(Pb)*Pa, where Pa and Pb are the corresponding cameras.
 *          h - The null vector of Pb
 *          A - Points from the first reconstruction. The homography will take A
 *              into B via HXa = Xb
 *          B - Points from the second reconstruction
 */
mat align_oneview(mat PBplusPA, vec h,
                  std::vector<PointStructure*> &XAs,
                  std::vector<PointStructure*> &XBs);

/**
 * Upgrade a projective reconstruction to a metric one given K.
 *
 * Given a projective reconstruction where all the cameras have the same
 * calibration K, find and apply the 4x4 homography T to bring the
 * reconstruction into a metric frame where it can be decomposed as:
 *
 * :   P = K[R|-RC]
 *
 * The routine directly modifies the reconstruction, and in the process
 * corrects for the camera calibration matrix. In theory after the routine
 * finishes the camera matricies should take the form:
 *
 * :   P = [R|-RC]
 *
 * though because of noise this will not be the case. Thus, when recovering the
 * R and C matricies, it is important to do a RQ decomposition of the first
 * three columns and correct for them:
 * 
 * :   Kp,R = rq(P[:,:3])
 * :   Pp  = Kp^-1 * P
 *
 * Then one can use the R to find the rotation, and the last three columns of
 * Pp to find the translation.
 *
 * Parameters:
 *   pr - Projective reconstruction to upgrade to metric
 *   K  - Camera calibration parameters
 */
mat find_euclidean_upgrade_calibrated(ProjectiveReconstruction &pr, mat K, bool flip);

TrackedSequence* read_from_file(const char *fn);
TrackedSequence* read_from_file(FILE* fid);
TrackedSequence *read_json_track(const char *fn);
TrackedSequence* json_to_tracked_sequence(Json *js);
void dump_to_file(TrackedSequence &ts, const char *fn);
// NOTE this MUST match a numbered trackedsequence.
void dump_reconstruction_to_file(ProjectiveReconstruction& pr, const char *fn);
void dump_reconstruction_to_file(ProjectiveReconstruction& pr, FILE* fid);
ProjectiveReconstruction *read_reconstruction_from_file(TrackedSequence& ts, const char *fn);
ProjectiveReconstruction *read_reconstruction_from_file(TrackedSequence& ts, FILE* fid);
void dump_many_reconstructions(
		const char* fn,
		std::vector<ProjectiveReconstruction*> &reconstructions);
void read_many_reconstructions(
		TrackedSequence& ts,
		const char *fn,
		std::vector<ProjectiveReconstruction*> *reconstructions);

// euclidean
void dump_euclidean_reconstruction_to_file(EuclideanReconstruction& pr, FILE* fid);
void dump_euclidean_reconstruction_to_file(EuclideanReconstruction& pr, const char *fn);
EuclideanReconstruction *read_euclidean_reconstruction_from_file(TrackedSequence& ts, const char *fn);
EuclideanReconstruction *read_euclidean_reconstruction_from_file(TrackedSequence& ts, FILE* fid);

ProjectiveReconstruction* reconstruct_hierarchical(TrackedSequence &ts, size_t ransac_rounds, int last_frame);
ProjectiveReconstruction* merge_hierarchical(
		TrackedSequence &ts,
		std::vector<Reconstruction*> &prs);

TrackedSequence* trim_tracks_shorter_than(int min_track_length);

/**
 * Force the first three colums of each camera to be a rotation matrix
 */
void force_orthogonal_rotation(ProjectiveReconstruction *pr);
EuclideanReconstruction *projective_to_euclidean(ProjectiveReconstruction* pr);
void apply_euler_rotation(mat &R, double phi, double theta, double psi);
void force_positive_depth(ProjectiveReconstruction *pr);
void reconstruct_sequential_subsets(TrackedSequence &ts, size_t ransac_rounds,
		int last_frame, std::vector<Reconstruction*> *subsets);
void reconstruct_keyframes(
		TrackedSequence &ts,
		size_t ransac_rounds,
		const std::vector<size_t> &keyframes,
		std::vector<Reconstruction*> *subsets,
		Config *config);
void reconstruct_keyframes_nview(
		TrackedSequence &ts,
		size_t ransac_rounds,
		const std::vector<size_t> &keyframes,
		std::vector<Reconstruction*> *subsets,
		Config *config);
void bundle_adjust_subsets(const
		std::vector<Reconstruction*> &subsets);

// uklt
void make_sigma_points(const mat &sigma, double meanx, double meany, mat *pts_out);

//======================================================================
//================= Linear Algebra =====================================
//======================================================================


mat opengl_to_normalized_k(const mat &ogl);

//======================================================================
//======================================================================
//======================================================================

} // namespace mv
#endif // _MV_H

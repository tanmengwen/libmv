#ifndef _FEATURE_H
#define _FEATURE_H

class Feature
{
public: 
	Feature();
	Frame *get_frame();
	int get_frame_number();
	int get_track_number();
	size_t get_number();
	Track *get_track();
	virtual ~Feature();
	virtual Json* dump_json();
	virtual string dump_cpp_ascii();
	void set_frame(Frame* frame);
	void set_frame_number(int n);
	void set_number(size_t n);
	void set_track(Track* parent);
private:
	Frame *frame;
	int frame_number;
	Track *track; 
	size_t number;
};

class UnscentedTransform;

class PointFeature : public Feature, public vec
{
public:
	PointFeature() : vec(3), sigma(2,2), ut(NULL) {
		sigma(0,0) = 1.0;
		sigma(0,1) = 0.0;
		sigma(1,0) = 0.0;
		sigma(1,1) = 1.0;
	}
	PointFeature(double x, double y) : vec(3), sigma(2,2), ut(NULL) 
		{set(x,y);}
	inline double x() {return (*this)[0];}
	inline double y() {return (*this)[1];}
	inline void set(double x, double y)
		{(*this)[0] = x; (*this)[1] = y; (*this)[2] = 1.0;}
	string dump_cpp_ascii() {
		char s[200];
		snprintf(s,200,"ts.add_point_feature(%d,%d, %g,%g);\n",
				get_frame_number(), get_track_number(),
				x(), y());
		return string(s);
	}
	virtual Json* dump_json();

	virtual ~PointFeature() {};
	mat sigma;
	mat sigma_inv_sqrt;
	UnscentedTransform *ut;
};

//class LineFeature : public Feature
//{ };

#endif // _FEATURE_H

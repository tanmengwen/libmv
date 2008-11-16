/* Python connection */

#include "mv.h"

#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle

#ifdef _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE // Get rid of very annoying warning
#endif

#include "num_util.h"
#include <iostream>

namespace mv {

namespace bp = boost::python;
namespace ar = boost::python::numeric;
namespace nbpl = num_util;

typedef ar::array npmat;


npmat make_npmat(size_t rows, size_t cols)
{
	std::vector<int> strides;
	strides.push_back(rows);
	strides.push_back(cols);
	npmat ret = nbpl::makeNum(strides, PyArray_DOUBLE);
	return ret;
}

double *get_npmat_data(npmat np)
{
	return (double*)nbpl::data(np);
}

class WrappedTrackedSequence : public TrackedSequence
{
public:
	ar::array get_point_features(int frame)
	{
		std::vector<int> strides;
		int n = (*this)[frame]->size();
		npmat ret_arr = make_npmat(n, 2);
		double *ret = get_npmat_data(ret_arr);

		for (int i=0; i<n; i++) {
			PointFeature *pf = (PointFeature*)(*(*this)[frame])[i];
			ret[2*i+0] = pf->x();
			ret[2*i+1] = pf->y();
		}

		return ret_arr;
	}

	ar::array get_track_matrix()
	{
		int n = tracks.size();
		int m = this->size();
		npmat ret_arr = make_npmat(n, m);
		double *ret = get_npmat_data(ret_arr);

		for (int i=0; i<n; i++) {
			int o = tracks[i]->size();
			for (int j=0; j<o; j++) {
				int fr = (*tracks[i])[j]->get_frame_number();
				ret[m*i+fr] = 1.0;
			}
		}

		return ret_arr;
	}

};

class WrappedProjectiveReconstruction : public ProjectiveReconstruction
{
public:
	bp::tuple get_frame_inliers(size_t frame)
	{
		assert(frame < cameras.size());
		bp::list measurements;
		bp::list reprojections;
		MeasurementVisitor mv(this);
		while(mv.next()) {
			if (mv.camera()->get_frame_number() != frame)
				continue; // inefficient... oh well

			PointFeature* pf = (PointFeature*)mv.measurement();
			measurements.append(bp::make_tuple((*pf)[0], (*pf)[1]));

			PointStructure* pt = (PointStructure*)mv.structure();
			PointFeature repro = pcam(frame)->project_point(pt);
			reprojections.append(bp::make_tuple(repro[0], repro[1]));
		}
		return bp::make_tuple(measurements, reprojections);
	}

	bp::list get_tracks() {
		bp::list bins[structure.size()];
		number_items();

		MeasurementVisitor mv(this);
		while(mv.next()) {
			size_t n = mv.structure()->number;
			PointFeature* pf = (PointFeature*)mv.measurement();
			bins[n].append(bp::make_tuple((*pf)[0], (*pf)[1]));
		}
		bp::list ret;
		for (size_t i=0; i<structure.size(); i++) {
			ret.append(bins[i]);
		}
		return ret;
	}

	void py_reconstruct(WrappedTrackedSequence &ts) {
		reconstruct(ts);
	}
};

BOOST_PYTHON_MODULE(_mv)
{ 
  import_array();
  ar::array::set_module_and_type("numpy", "ndarray");
  char docstring[] =  "libmv python bridge";
  bp::scope().attr("RCSID") = docstring;
  bp::scope().attr("__doc__") = docstring;

  bp::class_<WrappedTrackedSequence>("TrackedSequence")
	  .def("track_pgm_sequence", &WrappedTrackedSequence::track_pgm_sequence)
	  .def("get_point_features", &WrappedTrackedSequence::get_point_features)
	  .def("get_track_matrix", &WrappedTrackedSequence::get_track_matrix)
	  .def("enable_debug_output", &WrappedTrackedSequence::enable_debug_output)
  ;

  bp::class_<WrappedProjectiveReconstruction>("ProjectiveReconstruction")
	  .def("reconstruct", &WrappedProjectiveReconstruction::py_reconstruct)
	  .def("get_frame_inliers", &WrappedProjectiveReconstruction::get_frame_inliers)
	  .def("get_tracks", &WrappedProjectiveReconstruction::get_tracks)
  ;
}

} // namespace mv

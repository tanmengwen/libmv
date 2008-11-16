#include <json.h>
#include <testmv.h>
#include "json_numeric.h"
#include "test_gric_data.h"

// sadly this test is complicated and might even need testing itself...  i
// deeply regret some of my data structure choices now that we're later in the
// game. in particular, the 3-view 6-pt algorithm should work on just a double*
// and output to a double *P, double *Xs, int n where n is the number of
// reconstructions. then the current six-view code can be implemented in terms
// of that. then testing becomes much easier, and doesn't involve this
// complicated data structure dance.
static void load_tracks_from_json(
		const Json& js,
		std::vector<ThreeFrameTrack> *tfts) {
	mat xs[3];
	for (int i=0; i<3; i++) {
		json2mat(&js[0], &xs[i]);
	}
	size_t n = xs[0].size2();
	tfts->resize(n);
	for (size_t i=0; i<n; i++) {
		ThreeFrameTrack &tf = tfts->at(i);
		for (size_t j=0; j<3; j++)
			tf[j] = new PointFeature(xs[j](0,i), xs[j](1,i));
	}
}

static void cleanup(std::vector<ThreeFrameTrack> *tfts) {
	for (size_t i=0; i<tfts->size(); i++)
		for (int j=0; j<3; j++) 
			delete tfts->at(i)[j];
}

TEST() {
	(void) load_tracks_from_json;
	(void) gric_json;
	Json *js = json::Parse(gric_json);
	std::vector<ThreeFrameTrack> *tfts;

	load_tracks_from_json(js["PP"], tfts);

//	fit model HH;
//	fit model PPP;
//	calculate gric (data, fittedPPP)
//	calculate gric (data, fittedHH)
}

TEST(ppp_has_lower_gric_on_general_scene) {
//	calculate ppp gric (tfts)
//	calculate hh gric (tfts)
//	ppp < hh
}

TEST(hh_has_lower_gric_on_planar_scene) {
//	calculate ppp gric (tfts)
//	calculate hh gric (tfts)
//	hh < ppp
}

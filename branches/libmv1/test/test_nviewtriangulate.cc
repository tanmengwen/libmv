#include "testmv.h"
#include "nviewtriangulate.h"
#include "test_nviewtriangulate_data.h"

TEST(nview_triangluate_works_5views) {
	MATDATA(x);
	VECDATA(X);
	MATDATA(P0);
	MATDATA(P1);
	MATDATA(P2);
	MATDATA(P3);

	std::vector<mat*> cameras;
	cameras.push_back(&P0);
	cameras.push_back(&P1);
	cameras.push_back(&P2);
	cameras.push_back(&P3);

	vec X_result;
	nviewtriangulate(x, cameras, &X_result);
	AssertProjectiveEquals(X, X_result);
}

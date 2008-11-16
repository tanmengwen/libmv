#include "testmv.h"

void AllVerySmall(mat &H) {
	for (size_t i=0; i<H.size1(); i++) 
		for (size_t j=0; j<H.size2(); j++) 
			VerySmall(H(i,j));
}

void AllSmall(mat &H) {
	for (size_t i=0; i<H.size1(); i++) 
		for (size_t j=0; j<H.size2(); j++) 
			Small(H(i,j));
}

void AllVerySmall(vec &H) {
	for (size_t i=0; i<H.size(); i++) 
		VerySmall(H[i]);
}

void AllSmall(vec &H) {
	for (size_t i=0; i<H.size(); i++) 
		Small(H[i]);
}

void AssertProjectiveEquals(vec &a, vec &b) {
	vec ap = a;// / norm(a);
	vec bp = b;// / norm(b);
	ap /= norm_2(a);
	bp /= norm_2(b);
	assert (ap[0] != 0.0);
	assert (bp[0] != 0.0);
	if (ap[0] < 0)
		ap *= -1;
	if (bp[0] < 0)
		bp *= -1;
	vec tmp = ap-bp;
	AllSmall(tmp);
}

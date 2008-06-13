
#include "../Camera.h"
#include <vcl_iostream.h>

int main()
{
	Camera c;

	double p[] = {	1,0,1,0,
			1,1,0,1,
   			1,1,1,0 };
	vnl_double_3x4 P(p);

	c.set_P(P);

	vcl_cout << c.P << vcl_endl;
	vcl_cout << c.K << vcl_endl;

	// TODO test more class Camera and add asserts

	return 0;
}
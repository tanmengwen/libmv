#include "../MiddleburyData.h"

int main()
{
	MiddleburyData data;

	data.read("/home/paulinus/pro/mv/src/dense/tests/data", "templeR", true);

	vcl_cout << data.cameras[0].K << vcl_endl;
	vcl_cout << data.cameras[0].width << vcl_endl;
	vcl_cout << data.cameras[0].height << vcl_endl;

	return 0;
}


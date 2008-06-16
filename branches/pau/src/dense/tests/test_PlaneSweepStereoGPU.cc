#include "../MiddleburyData.h"
#include "../PlaneSweepStereoGPU.h"

int main(int argc, char *argv[])
{
	if(argc<4)
	{
		vcl_cout << "USAGE: test_PlaneSweepStereoGPU datadir dataname output" << vcl_endl;
		return 1;
	}

	MiddleburyData data;

	data.read(argv[1], argv[2], true);

	PlaneSweepStereoGPU ps;

	ps.set_key_image(data.images[0],data.cameras[0]);
	for(int i=0; i<data.nimages; i++)
	{
		if(i==data.nimages/2)
			ps.set_key_image(data.images[i],data.cameras[i]);
		else
			ps.add_reference_image(data.images[i],data.cameras[i]);
	}
	ps.set_bounding_box( data.bbox[0][0],data.bbox[0][1],data.bbox[0][2],
			     data.bbox[1][0],data.bbox[1][1],data.bbox[1][2] );

	ps.compute(30,PlaneSweepStereo::SSD,9);

	Image depthmap = ps.get_depthmap();

	for(int i=0;i<depthmap.size();i++)
		depthmap.top_left_ptr()[i] *= 255;

	Image_save(depthmap, argv[3]);


	return 0;
}

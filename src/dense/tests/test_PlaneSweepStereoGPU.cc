
#include <iostream>
#include <algorithm>

#include "../MiddleburyData.h"
#include "../PlaneSweepStereoGPU.h"

using namespace std;

int main(int argc, char *argv[]) {
  if(argc<4) {
          vcl_cout <<
"USAGE:\n"
"test_PlaneSweepStereoGPU datadir dataname output [correlation_type] [window_size]"
            << vcl_endl;
          return 1;
  }

  PlaneSweepStereo::CorrelationType correlation;
  correlation = PlaneSweepStereo::SD;
  if(argc>4) {
    if ( strcmp(argv[4],"SSD") == 0) {
      correlation = PlaneSweepStereo::SSD;
    } else if ( strcmp(argv[4],"ZNCC") == 0) {
      correlation = PlaneSweepStereo::ZNCC;
    }
  }

  int window_size = 1;
  if(argc>5) {
    window_size = atoi(argv[5]);
  }

  MiddleburyData data;
  data.read(argv[1], argv[2], true);

  PlaneSweepStereoGPU ps;
  for (int i = 0; i < data.nimages; i++) {
    if (i == data.nimages / 2)
      ps.set_key_image(data.images[i], data.cameras[i]);
    else
      ps.add_reference_image(data.images[i], data.cameras[i]);
  }
  ps.set_bounding_box(data.bbox[0][0],data.bbox[0][1],data.bbox[0][2],
                      data.bbox[1][0],data.bbox[1][1],data.bbox[1][2] );


  ps.compute(200, correlation, window_size);


  std::string depthmap_filename = std::string(argv[3]) + ".depthmap.pgm";
  Image depthmap = ps.get_depthmap();
  for(int i = 0; i < depthmap.size(); i++) {
    depthmap.top_left_ptr()[i] *= 255;
  }
  Image_save(depthmap, depthmap_filename.c_str());

  std::string confidence_filename = std::string(argv[3]) + ".confidence.pgm";
  Image confidence = ps.get_confidence();
  for(int i = 0; i < confidence.size(); i++) {
    confidence.top_left_ptr()[i] *= 255;
  }
  Image_save(confidence, confidence_filename.c_str());

  return 0;
}

#ifndef _RESECTION_RECONSTRUCTION_H
#define _RESECTION_RECONSTRUCTION_H
namespace mv {
class ProjectiveReconstruction;
void resection_reconstruction(ProjectiveReconstruction *pr, double thresh, int max_cameras=-1);
}
#endif // _RESECTION_RECONSTRUCTION_H

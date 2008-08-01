#ifndef __stereoviewer_h__
#define __stereoviewer_h__


#include <QGLViewer/qglviewer.h>
#include <vector>

#include "../dense/MiddleburyData.h"



using namespace std;

class MultiViewStereoViewer : public QGLViewer {
  Q_OBJECT

  MiddleburyData data;

  // parameters
  const char *dataDir, *dataName;

  bool texturesInited;
  bool must_lighting;
  float camera_size;

  // data
  vector<vector<GLubyte> > glimages;
  vector<GLuint> texID;
  int glimagessize;

public:
  MultiViewStereoViewer();

protected:
  void buildOpenglImages(int size=256);
  void initTextures();
  virtual void draw();
  void draw_cameras();
  void drawBoundingBox();
  virtual void init();

 public slots:
  void setDataset( const char dir[], const char name[]);
};

#endif //__stereoviewer_h__

#include "multi_view_stereo_viewer.h"

MultiViewStereoViewer::MultiViewStereoViewer()
  : QGLViewer() {
  glimagessize=0;
}

void MultiViewStereoViewer::buildOpenglImages(int size) {
  glimagessize = size;
  glimages.resize(data.nimages);
  for (int i = 0; i < data.nimages; ++i) {
    const Image &im = data.images[i];

    glimages[i].resize(size*size*3);

    for (int y = 0; y < size; ++y)
    for (int x = 0; x < size; ++x)
    for (int k = 0; k < 3; ++k) {
      float xx = float(x)*im.ni() / size;
      float yy = float(y)*im.nj() / size;
      glimages[i][(y*size+x)*3+k] = (GLubyte)im(xx,yy,k);
    }
  }
}

void MultiViewStereoViewer::initTextures() {
  texID.resize(data.nimages);
  glGenTextures(data.nimages, &texID[0]);
  for (int i = 0; i < data.nimages; ++i) {
    glBindTexture(GL_TEXTURE_2D, texID[i]);

    // select modulate to mix texture with color for shading
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
    // when texture area is small, bilinear filter the closest mipmap
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    // when texture area is large, bilinear filter the original
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    // the texture wraps over at the edges (repeat)
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

    glTexImage2D(GL_TEXTURE_2D, 0, 3, glimagessize, glimagessize,
                  0, GL_RGB, GL_UNSIGNED_BYTE, &glimages[i][0]);
  }
  texturesInited=true;
}

void MultiViewStereoViewer::draw() {
  draw_cameras();
  drawBoundingBox();
}

static void Rt2gl(GLdouble res[4][4], const Camera &cam ) {
  const vnl_double_3x3 &R = cam.R;
  const vnl_double_3 &t = cam.t;

  //  (R'|t')  =  (RT|-RT*t)
  res[0][0] = R(0,0); res[1][0] = R(1,0); res[2][0] = R(2,0);
  res[0][1] = R(0,1); res[1][1] = R(1,1); res[2][1] = R(2,1);
  res[0][2] = R(0,2); res[1][2] = R(1,2); res[2][2] = R(2,2);
  res[0][3] =      0; res[1][3] =      0; res[2][3] =      0;

  res[3][0] = - R(0,0) * t(0) - R(1,0) * t(1) - R(2,0) * t(2);
  res[3][1] = - R(0,1) * t(0) - R(1,1) * t(1) - R(2,1) * t(2);
  res[3][2] = - R(0,2) * t(0) - R(1,2) * t(1) - R(2,2) * t(2);
  res[3][3] = 1;
}

void MultiViewStereoViewer::draw_cameras() {
  //draw the cameras
  glPointSize(3);
  glDisable(GL_LIGHTING);
  glBegin(GL_POINTS);
  for (int i = 0; i < data.nimages; ++i) {
    glColor3f(0.6,1.,0.6);
    glColor3f(1.,0.6,0.6);
    glVertex3dv(&data.cameras[i].ocenter[0]);
  }
  glEnd();
  glEnable(GL_LIGHTING);


  if(!texturesInited) initTextures();
  for (int i = 0; i < data.nimages; ++i) {
    Camera &cam = data.cameras[i];

    glPushMatrix();
    GLdouble tmp[4][4];
    Rt2gl(tmp, cam);
    glMultMatrixd(&tmp[0][0]);

    glColor4f(1.,1.,1.,1.);

    if(texturesInited) {
      glEnable(GL_TEXTURE_2D);                        // Enable Texture Mapping
      glBindTexture(GL_TEXTURE_2D, texID[i]);         // Select Our Texture
    }
    float z = camera_size;
    int width = cam.width;
    int height = cam.height;

    vnl_double_3 top_left =
        data.cameras[i].intrinsic_pixel_direction(-.5,-.5);
    top_left *= z / top_left[2];
    vnl_double_3 bottom_left =
        data.cameras[i].intrinsic_pixel_direction(-.5, height -.5);
    bottom_left *= z / bottom_left[2];
    vnl_double_3 bottom_right =
        data.cameras[i].intrinsic_pixel_direction(width - .5, height - .5);
    bottom_right *= z / bottom_right[2];
    vnl_double_3 top_right =
        data.cameras[i].intrinsic_pixel_direction(width - .5, -.5);
    top_right *= z / top_right[2];

    glNormal3f(0,0,-1);
    glBegin(GL_QUADS); {
      glTexCoord2f(0.0f, 0.0f);
      glVertex3dv(&top_left[0]);

      glTexCoord2f(0.0f, 1.0f);
      glVertex3dv(&bottom_left[0]);

      glTexCoord2f(1.0f, 1.0f);
      glVertex3dv(&bottom_right[0]);

      glTexCoord2f(1.0f, 0.0f);
      glVertex3dv(&top_right[0]);
    } glEnd();

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

//     glBegin(GL_LINES);
//       glColor4f(0.5f, 0.4f, 1.0f, 1.0f);
//       glVertex3f(0,0,0);
//       glVertex3dv(&top_left[0]);
//       glVertex3f(0,0,0);
//       glVertex3dv(&bottom_left[0]);
//       glVertex3f(0,0,0);
//       glVertex3dv(&bottom_right[0]);
//       glVertex3f(0,0,0);
//       glVertex3dv(&top_right[0]);
//     glEnd();

    if (axisIsDrawn()) {
      QGLViewer::drawAxis(camera_size / 2);
    }

    glPopMatrix();
  }
}

void MultiViewStereoViewer::drawBoundingBox() {
  double (&bbox)[2][3] = data.bbox;
  glBegin(GL_LINES);
  glColor3f(0,1.,0);
  for(int x=0;x<2;x++) for(int y=0;y<2;y++) for(int z=0;z<2;z++)
    glVertex3d( bbox[x][0], bbox[y][1], bbox[z][2] );
  for(int y=0;y<2;y++) for(int z=0;z<2;z++) for(int x=0;x<2;x++)
    glVertex3d( bbox[x][0], bbox[y][1], bbox[z][2] );
  for(int z=0;z<2;z++) for(int x=0;x<2;x++) for(int y=0;y<2;y++)
    glVertex3d( bbox[x][0], bbox[y][1], bbox[z][2] );
  glEnd();
}

void MultiViewStereoViewer::init() {
  if (!data.read(dataDir, dataName, true)) {
    printf("missing data files\n");
    return;
  }

  buildOpenglImages();

  setSceneCenter( qglviewer::Vec( (data.bbox[0][0]+data.bbox[1][0])/2.,
                                  (data.bbox[0][1]+data.bbox[1][1])/2.,
                                  (data.bbox[0][2]+data.bbox[1][2])/2.) );
  float rad=0;
  for (int i = 0; i < data.nimages; ++i) {
    qglviewer::Vec O(data.cameras[i].ocenter[0],
                    data.cameras[i].ocenter[1],
                    data.cameras[i].ocenter[2] );
    if (rad < (sceneCenter()-O).norm())
      rad = (sceneCenter()-O).norm();
  }

  setSceneRadius(rad * 1.5);
  showEntireScene();
  camera_size = pow(10, float(-6) / 10) * sceneRadius();
}

void MultiViewStereoViewer::setDataset( const char dir[], const char name[]) {
  dataDir = strdup(dir);
  dataName = strdup(name);
}



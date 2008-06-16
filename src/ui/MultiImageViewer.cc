#include <QtGui>
#include <QtOpenGL>
#include <math.h>
#include "MultiImageViewer.h"



MultiImageViewer::MultiImageViewer(QWidget *parent)
: QGLWidget(parent)
{
	tx = 0;
	ty = 0;
	zoom = 1;
}

MultiImageViewer::~MultiImageViewer()
{
	makeCurrent();
}


void MultiImageViewer::loadImageFile( const QStringList &filenames )
{
	int inum = filenames.size();

	// allocate a texture name
	GLuint textureID[inum];
	glGenTextures( inum, &textureID[0] );
	si.resize(inum);
	for(int i=0;i<inum;i++)
		si[i].textureID = textureID[i];

	// read image files and bind textures
	for(int l=0;l<inum;l++)
	{
		QImage im( filenames[l] );
		si[l].width = im.width();
		si[l].height = im.height();
		unsigned char *data = new unsigned char[si[l].width*si[l].height*4];

		for(int i=0;i<si[l].height;i++)
			for(int j=0;j<si[l].width;j++)
			{
				data[4*(i*si[l].width+j) + 0] = im.bits()[4*(i*si[l].width+j) + 2];
				data[4*(i*si[l].width+j) + 1] = im.bits()[4*(i*si[l].width+j) + 1];
				data[4*(i*si[l].width+j) + 2] = im.bits()[4*(i*si[l].width+j) + 0];
			}

		// select our current texture
		glBindTexture( GL_TEXTURE_2D, si[l].textureID );
		// select modulate to mix texture with color for shading
		glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
		// when texture area is small, bilinear filter the closest mipmap
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST );
		// when texture area is large, bilinear filter the original
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
		// the texture wraps over at the edges (repeat)
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

		// build our texture mipmaps
		gluBuild2DMipmaps( GL_TEXTURE_2D, GL_RGBA, si[l].width, si[l].height,
			   GL_RGBA, GL_UNSIGNED_BYTE, data );

		delete [] data;

		if(l==0) {
			si[0].posx = 0;
			si[0].posy = 0;
		} else {
			si[l].posx = si[l-1].posx + si[l-1].width + 10;
			si[l].posy = si[l-1].posy;
		}
	}
}

QSize MultiImageViewer::minimumSizeHint() const
{
	return QSize(50, 50);
}

QSize MultiImageViewer::sizeHint() const
{
	return QSize(800, 400);
}
void MultiImageViewer::setTransformation(float tx_, float ty_, float zoom_)
{
	tx = tx_;
	ty = ty_;
	zoom = zoom_;
	updateGL();
}

void MultiImageViewer::planeFromScreen( float &xi, float &yi, const float xw, const float yw )
{
	xi = zoom * xw + tx;
	yi = zoom * yw + ty;
}
void MultiImageViewer::screenFromPlane(  float &xw, float &yw, const float xi, const float yi )
{
	xw = (xi - tx) / zoom;
	yw = (yi - ty) / zoom;
}


void MultiImageViewer::initializeGL()
{
	glClearColor(0,0,0,1);
	glShadeModel(GL_FLAT);


	//alba
	QStringList pathImage = QFileDialog::getOpenFileNames(this,"Select Images", "", "Image Files (*.png)");
	loadImageFile(pathImage);
}


void MultiImageViewer::setUpGlCamera()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float x0,y0, x1,y1;
	planeFromScreen(x0,y0,0,0);
	planeFromScreen(x1,y1,width(),height());
	glOrtho(x0,x1, y1,y0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
}

void MultiImageViewer::drawImages()
{
	for( int i=0; i<si.size(); i++ ) {
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(si[i].posx,si[i].posy,0);

		glEnable( GL_TEXTURE_2D );
		glBindTexture( GL_TEXTURE_2D, si[i].textureID );
		glBegin(GL_QUADS);
			glTexCoord2d(0,0); glVertex2d(0,0);
			glTexCoord2d(1,0); glVertex2d(si[i].width,0);
			glTexCoord2d(1,1); glVertex2d(si[i].width,si[i].height);
			glTexCoord2d(0,1); glVertex2d(0,si[i].height);
		glEnd();
		glDisable( GL_TEXTURE_2D );

		glPopMatrix();
	}
}

void MultiImageViewer::paintGL()
{
	setUpGlCamera();
	drawImages();
}

void MultiImageViewer::resizeGL(int width, int height)
{
	glViewport(0, 0, width,height);
}

void MultiImageViewer::mousePressEvent(QMouseEvent *event)
{
	lastPos = event->pos();
}

void MultiImageViewer::mouseMoveEvent(QMouseEvent *event)
{
	float x0,y0, x1,y1;
	planeFromScreen(x0, y0, lastPos.x(), lastPos.y() );
	planeFromScreen(x1, y1, event->x(), event->y() );

	if (event->buttons() & Qt::LeftButton) {
		setTransformation( tx + x0 - x1, ty + y0 - y1, zoom );
	}
	lastPos = event->pos();
}


void MultiImageViewer::wheelEvent(QWheelEvent *event)
{
	// zoom so that the image point under the cursor does not move
	const float newzoom = zoom*pow(1.001, event->delta());
	const float newtx = (zoom - newzoom) * width() / 2. + tx;
	const float newty = (zoom - newzoom) * height() / 2. + ty;

	setTransformation(newtx,newty,newzoom);
	//~ printf("tx=%g  ty=%g  zoom = %g\n",tx,ty,zoom);
}

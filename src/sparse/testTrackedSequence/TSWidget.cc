#include <QtGui>
#include <QtOpenGL>
#include <math.h>
#include "TSWidget.h"



TSWidget::TSWidget(QWidget *parent)
: MultiImageViewer(parent)
{
	tx = 0;
	ty = 0;
	zoom = 1;
}

TSWidget::~TSWidget()
{
}


void TSWidget::loadImageFile( const QStringList &filenames )
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
	}
}

QSize TSWidget::minimumSizeHint() const
{
	return QSize(50, 50);
}

QSize TSWidget::sizeHint() const
{
	return QSize(800, 400);
}
void TSWidget::setTransformation(float tx_, float ty_, float zoom_)
{
	tx = tx_;
	ty = ty_;
	zoom = zoom_;
	updateGL();
}

void TSWidget::imageFromScreen( float &xi, float &yi, const float xw, const float yw )
{
	xi = zoom * xw + tx;
	yi = zoom * yw + ty;
}
void TSWidget::screenFromImage(  float &xw, float &yw, const float xi, const float yi )
{
	xw = (xi - tx) / zoom;
	yw = (yi - ty) / zoom;
}


void TSWidget::initializeGL()
{
	glClearColor(0,0,0,1);
	glShadeModel(GL_FLAT);


	QStringList pathImage = QFileDialog::getOpenFileNames(this,"Select Images", "", "Image Files (*.png)");
	loadImageFile(pathImage);

	int inum = pathImage.size();

	std::vector<std::string> fn;
	for(int i=0;i<inum;i++)
		fn.push_back(std::string(pathImage[i].toAscii().data()));

	ts.load_frames(fn);
	ts.compute_sift_features();
	ts.compute_two_view_matches();
	ts.compute_track_from_matches();
}



void TSWidget::paintGL()
{
	// set up camera
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float x0,y0, x1,y1;
	imageFromScreen(x0,y0,0,0);
	imageFromScreen(x1,y1,width(),height());
	glOrtho(x0,x1, y1,y0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	int inum = ts.frames.size();

	si[0].posx = si[0].posy = 0;
	for(int i=1;i<inum;i++) {
		si[i].posx = si[i-1].posx + si[i-1].width;
		si[i].posy = si[i-1].posy + si[i-1].height;
	}

	// draw the images
	for( int i=0; i<inum; i++ ) {
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


		glPointSize(3);
		for(unsigned int j=0; j<ts.frames[i].features.size(); j++)
		{
			SiftFeature &sf = ts.frames[i].features[j];

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glTranslatef(sf.x,sf.y,0);
			glRotatef(180*sf.angle/M_PI,0,0,1);

			glBegin(GL_LINES);
			glVertex2f( - sf.scale, - sf.scale);
			glVertex2f( + sf.scale, - sf.scale);

			glVertex2f( + sf.scale, - sf.scale);
			glVertex2f( + sf.scale, + sf.scale);

			glVertex2f( + sf.scale, + sf.scale);
			glVertex2f( - sf.scale, + sf.scale);

			glVertex2f( - sf.scale, + sf.scale);
			glVertex2f( - sf.scale, - sf.scale);
			glEnd();
			glPopMatrix();
		}

		glPopMatrix();
	}


/*
	glBegin(GL_LINES);
	for(unsigned int m=0; m<candidateMatches.size(); m++)
	{
		SiftFeature &left = si[0].features[candidateMatches[m].idxFeatureImage0];
		SiftFeature &right= si[1].features[candidateMatches[m].idxFeatureImage1];
		glVertex2f(si[0].posx +  left.x, si[0].posy +  left.y);
		glVertex2f(si[1].posx + right.x, si[1].posy + right.y);
	}
	glEnd();
*/
	//draw robust matches
	glColor4f(.3,1,.3,1);
	glBegin(GL_LINES);
	int nRobustMatches = 0;
	for(std::list<PointTrack>::iterator i=ts.tracks.begin();
		i!=ts.tracks.end(); i++ )
	{
		int isFullTrack = 1;
		for(int j=0; j<inum; j++)
			if(!i->track_indices[&ts.frames[j]])
			{
				isFullTrack = 0;
				break;
			}


		if(isFullTrack) {
			nRobustMatches++;
			for(int j=1; j<inum; j++)
			{
				SiftFeature &left = ts.frames[j-1].features[i->track_indices[&ts.frames[j-1]]];
				SiftFeature &right= ts.frames[j].features[i->track_indices[&ts.frames[j]]];
				glVertex2f(si[j-1].posx +  left.x, si[j-1].posy +  left.y);
				glVertex2f(si[j].posx + right.x, si[j].posy + right.y);
			}
		}
	}
	glEnd();


	glColor4f(1,1,0,1);
	QString messageStart("hi ha ");
	QString num("");
	num.setNum(nRobustMatches);
	QString messageEnd(" tracks");
	renderText(10,10,  messageStart + num + messageEnd);
}

void TSWidget::resizeGL(int width, int height)
{
	glViewport(0, 0, width,height);
}

void TSWidget::mousePressEvent(QMouseEvent *event)
{
	lastPos = event->pos();
}

void TSWidget::mouseMoveEvent(QMouseEvent *event)
{
	float x0,y0, x1,y1;
	imageFromScreen(x0, y0, lastPos.x(), lastPos.y() );
	imageFromScreen(x1, y1, event->x(), event->y() );

	if (event->buttons() & Qt::LeftButton) {
		setTransformation( tx + x0 - x1, ty + y0 - y1, zoom );
	}
	lastPos = event->pos();
}


void TSWidget::wheelEvent(QWheelEvent *event)
{
	// zoom so that the image point under the cursor does not move
	const float newzoom = zoom*pow(1.001, event->delta());
	const float newtx = (zoom - newzoom) * event->x() + tx;
	const float newty = (zoom - newzoom) * event->y() + ty;

	setTransformation(newtx,newty,newzoom);
	//~ printf("tx=%g  ty=%g  zoom = %g\n",tx,ty,zoom);
}

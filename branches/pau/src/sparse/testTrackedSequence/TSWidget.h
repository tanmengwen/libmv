#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <vector>

#include "sparse/Frame.h"
#include "sparse/TrackedSequence.h"
#include "ui/MultiImageViewer.h"



class TSWidget : public MultiImageViewer
{
	Q_OBJECT

	struct OnScreenImage
	{
		GLuint textureID;
		int width, height;
		float posx, posy;
	};


	public:
		TSWidget(QWidget *parent = 0);
		~TSWidget();

		QSize minimumSizeHint() const;
		QSize sizeHint() const;

	public slots:
		void loadImageFile( const QStringList &filenames );
		void setTransformation(float tx_, float ty_, float zoom_);

	//	void computeCandidateMatches();
	//	void computeFundametalMatrixRANSAC();

	protected:
		void initializeGL();
		void paintGL();
		void resizeGL(int width, int height);
		void mousePressEvent(QMouseEvent *event);
		void mouseMoveEvent(QMouseEvent *event);
		void wheelEvent(QWheelEvent *event);

		void imageFromScreen( float &xi, float &yi, const float xw, const float yw );
		void screenFromImage(  float &xw, float &yw, const float xi, const float yi );


	private:
		//SiftImage si[2];
		std::vector<OnScreenImage> si;

		TrackedSequence ts;

		float tx;  /// top left corner of the window in image coordinates
		float ty;
		float zoom;     /// window pixels per image pixel

		QPoint lastPos;
};

#endif

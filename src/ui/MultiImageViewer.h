#ifndef __MultiImageViewer_h__
#define __MultiImageViewer_h__

#include <QGLWidget>
#include <vector>

// A widget displaying multiple images on a plane.
// Dragging moves the plane.
// Scrolling zooms.
// Shift-dragging moves the selected image.
// Shift-scrolling scales the selected image.
class MultiImageViewer : public QGLWidget
{
	Q_OBJECT

	struct OnScreenImage
	{
		GLuint textureID;
		int width, height;
		float posx, posy;
		float scale;
	};


	public:
		MultiImageViewer(QWidget *parent = 0);
		~MultiImageViewer();

		QSize minimumSizeHint() const;
		QSize sizeHint() const;

	public slots:
		void loadImageFile( const QStringList &filenames );
		void setTransformation(float tx_, float ty_, float zoom_);

	protected:
		void initializeGL();
		void paintGL();
		void resizeGL(int width, int height);
		void mousePressEvent(QMouseEvent *event);
		void mouseMoveEvent(QMouseEvent *event);
		void wheelEvent(QWheelEvent *event);
		void setUpGlCamera();
		void drawImages();

		// coordinate systems
		void planeFromScreen( float &xi, float &yi, const float xw, const float yw );
		void screenFromPlane(  float &xw, float &yw, const float xi, const float yi );


	private:
		std::vector<OnScreenImage> si;

		float tx;  /// top left corner of the window in image coordinates
		float ty;
		float zoom;     /// window pixels per image pixel

		QPoint lastPos;
};

#endif //__MultiImageViewer_h__

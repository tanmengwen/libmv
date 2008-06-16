#include <QApplication>
#include <QFileDialog>
#include <QList>
#include <QImage>

#include "../MultiImageViewer.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);


	MultiImageViewer w;
	w.resize(600,400);
	w.show();

	QStringList imagePaths = QFileDialog::getOpenFileNames(
			&w, "Load Multiple Images", "", "Image Files (*.png *.jpg)");

	for(int i=0; i<imagePaths.size(); i++)
		w.addImage(QImage(imagePaths[i]));

	return app.exec();
}
